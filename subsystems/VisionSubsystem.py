"""VisionSubsystem: Wraps PhotonVision for AprilTag detection and pose estimation.

This subsystem provides:
- Integration with PhotonVision camera for AprilTag detection
- Pose estimation updates to the swerve drivetrain's pose estimator
- Ambiguity filtering to prevent pose "teleporting"
- Helper methods to get field-relative poses of specific AprilTags
- PhotonVision simulation support for testing without hardware

DESIGN DECISIONS:
- Ambiguity filtering (>0.2 threshold) provides sufficient quality control for most scenarios
- Additional filtering (jump detection, Z-axis bounds) for robust pose estimation
- Camera connection errors are caught by _safe_get_latest_result() to prevent loop overruns from _versionCheck() warnings
- The robot continues to operate using odometry-only if vision fails (graceful degradation)
- Simulation uses VisionSystemSim to approximate real PhotonVision behavior
"""

from commands2 import Subsystem
from ntcore import NetworkTableInstance
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import SmartDashboard, Timer, RobotBase
from wpimath.geometry import (
    Transform3d,
    Translation3d,
    Rotation3d,
    Pose2d,
    Pose3d,
    Rotation2d,
)

from typing import Optional

from vision_metrics.constants import (
    CAMERA_NAME,
    ROBOT_TO_CAM_X,
    ROBOT_TO_CAM_Y,
    ROBOT_TO_CAM_Z,
    AMBIGUITY_THRESHOLD,
    Z_AXIS_THRESHOLD,
    TILT_THRESHOLD_DEG,
    JUMP_THRESHOLD_M,
)
from vision_metrics.pose_estimator import (
    EstimationModeTracker,
    get_estimated_pose_with_fallback,
)


class VisionSubsystem(Subsystem):
    """
    Subsystem that manages AprilTag vision processing using PhotonVision.

    This subsystem:
    - Processes camera data from PhotonVision
    - Updates the robot's pose estimator with vision measurements
    - Filters ambiguous detections to maintain pose accuracy
    - Provides helper methods for target pose queries
    - Supports simulation via VisionSystemSim for testing without hardware
    """

    # Robot-to-Camera transform built from shared constants
    ROBOT_TO_CAMERA = Transform3d(
        Translation3d(ROBOT_TO_CAM_X, ROBOT_TO_CAM_Y, ROBOT_TO_CAM_Z),
        Rotation3d(0.0, 0.0, 0.0),
    )

    def __init__(self, drivetrain):
        """
        Initialize the VisionSubsystem.

        Args:
            drivetrain: The CommandSwerveDrivetrain instance for pose estimation updates
        """
        super().__init__()

        self._drivetrain = drivetrain

        # Initialize PhotonVision camera
        self._camera = PhotonCamera(CAMERA_NAME)

        # Get the AprilTag field layout for the current season
        # For 2026 Rebuilt season, there are TWO field layouts:
        #   k2026RebuiltWelded - For welded field construction (more common)
        #   k2026RebuiltAndyMark - For Andymark field construction
        # IMPORTANT: Ensure PhotonVision coprocessor uses the SAME layout!
        self._field_layout = AprilTagFieldLayout.loadField(
            AprilTagField.k2026RebuiltWelded
        )

        # Initialize PhotonPoseEstimator
        # Uses multi-tag estimation when multiple tags are visible for better accuracy
        self._pose_estimator = PhotonPoseEstimator(
            self._field_layout,
            self.ROBOT_TO_CAMERA,
        )

        # Track last valid vision update time for diagnostics
        self._last_valid_update_time = 0.0

        self._last_processed_timestamp = 0.0

        self._last_valid_pose: Optional[Pose2d] = None

        self._mode_tracker = EstimationModeTracker()

        # Publish pose provider status ("CV" or "ODO") to the Pose NT table
        pose_table = NetworkTableInstance.getDefault().getTable("Pose")
        self._provider_pub = pose_table.getStringTopic("provider").publish()

        # Defensive camera access tracking
        self._camera_error_logged = False
        self._consecutive_camera_failures = 0
        self._last_camera_error_time = 0.0

        # Throttle SmartDashboard publishing to every Nth cycle (~200ms at 50Hz)
        self._publish_interval = 10
        self._cycle_count = 0

        # Throttle VisionSystemSim.update() — real cameras run at ~30fps,
        # so updating every 3rd cycle (~17Hz) is sufficient and avoids overruns.
        self._sim_update_interval = 3
        self._sim_cycle_count = 0

        # Cached camera result for frame consistency.
        # periodic() populates this once per cycle; helper methods read from it
        # so that pose estimation and target tracking see the same frame.
        self._cached_result = None

        # Shared tracking state written by VisionTrackTargetPair, read by ShooterCommand.
        # Replaces SmartDashboard-based data passing between the two commands.
        self._target_distance: float = 0.0
        self._heading_error: float = 180.0

        # Simulation setup (only initialized in simulation mode)
        self._vision_sim: Optional[VisionSystemSim] = None
        self._camera_sim: Optional[PhotonCameraSim] = None
        self._simulation_initialized = False
        self._simulation_init_attempted = False

    def _initialize_simulation(self):
        """Initialize PhotonVision simulation components."""
        try:
            from photonlibpy.simulation import VisionSystemSim, PhotonCameraSim, SimCameraProperties

            # Create vision system simulation
            self._vision_sim = VisionSystemSim("main_vision")

            # Create camera simulation - use default properties
            # Default: 960x720 resolution, 90 degree FOV, no noise
            # This avoids API compatibility issues with setCalibration
            camera_props = SimCameraProperties()

            # Try to set properties if they exist (handle API differences)
            try:
                camera_props.setFPS(30)
            except AttributeError:
                pass  # Use default FPS

            # Create camera sim - use simpler constructor if available
            try:
                self._camera_sim = PhotonCameraSim(self._camera, camera_props)
            except TypeError:
                # Fallback: create without properties (use all defaults)
                self._camera_sim = PhotonCameraSim(self._camera)

            # Add camera to vision system with robot-to-camera transform
            self._vision_sim.addCamera(self._camera_sim, self.ROBOT_TO_CAMERA)

            # Add AprilTags to simulation
            self._vision_sim.addAprilTags(self._field_layout)

            self._simulation_initialized = True
            SmartDashboard.putBoolean("Vision/SimulationActive", True)
            print("[Vision] Simulation initialized successfully")

        except Exception as e:
            import traceback

            print(f"[Vision] Failed to initialize simulation: {e}")
            traceback.print_exc()
            SmartDashboard.putBoolean("Vision/SimulationActive", False)
            SmartDashboard.putString("Vision/SimulationError", str(e)[:200])

    def simulationPeriodic(self):
        """
        Called periodically during simulation.
        Updates the vision simulation with current robot pose.
        Throttled to every Nth cycle because VisionSystemSim.update()
        is expensive (~30-40ms in Python) and a real camera only runs at ~30fps.
        """
        if not RobotBase.isSimulation():
            return

        # Defer simulation initialization to first periodic call
        if not self._simulation_initialized and not self._simulation_init_attempted:
            self._initialize_simulation()
            self._simulation_init_attempted = True

        if not self._simulation_initialized:
            return

        # Throttle the expensive raycasting update
        self._sim_cycle_count += 1
        if self._sim_cycle_count < self._sim_update_interval:
            return
        self._sim_cycle_count = 0

        try:
            robot_pose = self._get_sim_robot_pose()

            if robot_pose is not None:
                # Update vision system with robot pose
                # This triggers target detection and publishes to NT4
                self._vision_sim.update(robot_pose)

        except Exception as e:
            SmartDashboard.putString("Vision/Sim/Error", str(e))

    def _get_sim_robot_pose(self) -> Optional[Pose3d]:
        """
        Get the current simulated robot pose from drivetrain.

        Returns:
            Pose3d of robot in field coordinates, or None if unavailable
        """
        try:
            # Try to get sim pose from drivetrain if it has simulation support
            if hasattr(self._drivetrain, "get_sim_pose"):
                return self._drivetrain.get_sim_pose()
            elif hasattr(self._drivetrain, "getSimPose"):
                return self._drivetrain.getSimPose()
            else:
                # Fallback: use current estimated pose (less accurate but functional)
                pose_2d = self._drivetrain.get_state().pose
                return Pose3d(
                    pose_2d.x,
                    pose_2d.y,
                    0.0,  # Z = 0 on ground
                    Rotation3d(0.0, 0.0, pose_2d.rotation().radians()),
                )
        except Exception as e:
            SmartDashboard.putString("Vision/Sim/PoseError", str(e))
            return None

    def _should_publish(self) -> bool:
        """Return True every Nth cycle to throttle SmartDashboard traffic."""
        self._cycle_count += 1
        if self._cycle_count >= self._publish_interval:
            self._cycle_count = 0
            return True
        return False

    def _safe_get_latest_result(self):
        """Safely get the latest camera result, returning None on failure.

        Throttles error logging to once per 5 seconds to avoid console spam
        from _versionCheck() warnings that cause loop overruns.
        """
        try:
            result = self._camera.getLatestResult()
            if self._consecutive_camera_failures > 0:
                print(
                    f"[Vision] Camera recovered after "
                    f"{self._consecutive_camera_failures} failures"
                )
                self._consecutive_camera_failures = 0
                self._camera_error_logged = False
            return result
        except Exception as e:
            self._consecutive_camera_failures += 1
            now = Timer.getFPGATimestamp()
            if not self._camera_error_logged or (now - self._last_camera_error_time > 5.0):
                print(f"[Vision] Camera error ({self._consecutive_camera_failures}x): {e}")
                self._camera_error_logged = True
                self._last_camera_error_time = now
            return None

    def periodic(self):
        """
        Called periodically by the CommandScheduler.
        Processes vision data and updates pose estimator.
        """
        publish = self._should_publish()

        result = self._safe_get_latest_result()
        self._cached_result = result  # Cache for helper methods within this cycle

        if result is None:
            self._provider_pub.set("ODO")
            if publish:
                SmartDashboard.putBoolean("Vision/CameraConnected", False)
                SmartDashboard.putString("Vision/RejectReason", "Camera error")
                SmartDashboard.putBoolean("Vision/HasTargets", False)
                SmartDashboard.putNumber(
                    "Vision/ConsecutiveFailures", self._consecutive_camera_failures
                )
            return

        self._provider_pub.set("CV" if result.hasTargets() else "ODO")

        # Handle simulation mode - use simulated camera results
        if RobotBase.isSimulation():
            self._periodic_sim()
        else:
            self._periodic_real(publish, result)

    def _periodic_sim(self):
        """Handle periodic updates in simulation mode.

        NOTE: Vision sim update is handled by simulationPeriodic() which the
        framework calls automatically — no need to duplicate it here.

        # In simulation, we skip pose estimation updates from camera
        # because we're using the simulated camera data instead
        # Just publish that we're in simulation mode
        """
        pass

    def _periodic_real(self, publish: bool, result):
        """Handle periodic updates in real robot mode.

        Args:
            publish: If True, update SmartDashboard diagnostics this cycle.
            result: PhotonPipelineResult from camera (obtained in periodic()).
        """
        if publish:
            SmartDashboard.putBoolean("Vision/CameraConnected", self._camera.isConnected())

        # Check if we have any targets
        if not result.hasTargets():
            if publish:
                SmartDashboard.putBoolean("Vision/HasTargets", False)
                SmartDashboard.putNumber("Vision/TargetCount", 0)
        else:
            if publish:
                SmartDashboard.putBoolean("Vision/HasTargets", True)
                SmartDashboard.putNumber("Vision/TargetCount", len(result.getTargets()))
            # Update pose estimator with latest result (always runs)
            self._update_pose_estimator(result, publish)
            if publish:
                multi_tag_active = len(result.getTargets()) > 1
                SmartDashboard.putNumber(
                    "Vision/EstimationMode", 1 if multi_tag_active else 0
                )

        if publish:
            if self._last_valid_update_time > 0:
                time_since_update = Timer.getFPGATimestamp() - self._last_valid_update_time
                SmartDashboard.putNumber("Vision/TimeSinceLastUpdate", time_since_update)
            else:
                SmartDashboard.putNumber("Vision/TimeSinceLastUpdate", -1.0)

    def _update_pose_estimator(self, result, publish: bool = False):
        """
        Update the drivetrain's pose estimator with vision measurements.

        Filtering and pose feeding run every cycle. SmartDashboard diagnostics
        are only published when *publish* is True (throttled by caller).

        Args:
            result: PhotonPipelineResult from camera
            publish: Whether to write diagnostics to SmartDashboard this cycle
        """
        estimation = get_estimated_pose_with_fallback(self._pose_estimator, result)

        if estimation is None:
            if publish:
                SmartDashboard.putBoolean("Vision/ValidPose", False)
                SmartDashboard.putString("Vision/RejectReason", "Pose estimation failed")
            return

        estimated_pose = estimation.estimated_pose
        vision_pose_3d = estimated_pose.estimatedPose
        timestamp = estimated_pose.timestampSeconds

        if publish:
            SmartDashboard.putBoolean("Vision/MultiTagMode", estimation.is_multi_tag)
            SmartDashboard.putNumber("Vision/TagsUsed", len(estimation.targets_used))
            strategy = "MultiTag" if estimation.is_multi_tag else "SingleTag"
            SmartDashboard.putString("Vision/EstimationStrategy", strategy)
            tag_ids_str = ",".join(str(id) for id in estimation.tag_ids)
            SmartDashboard.putString("Vision/TagIDs", tag_ids_str)

        changed, message = self._mode_tracker.check_mode_change(
            estimation.is_multi_tag, estimation.tag_ids
        )
        if changed and message:
            encoded = message.encode('latin-1', errors='replace')
            print(f"[Vision] {encoded}")

        best_target = result.getBestTarget()
        if best_target is not None:
            ambiguity = best_target.getPoseAmbiguity()
            if publish:
                SmartDashboard.putNumber("Vision/BestTargetAmbiguity", ambiguity)

            if ambiguity > AMBIGUITY_THRESHOLD:
                if publish:
                    SmartDashboard.putBoolean("Vision/ValidPose", False)
                    SmartDashboard.putString(
                        "Vision/RejectReason", f"High ambiguity: {ambiguity:.3f}"
                    )
                return

        # FILTERING: Freshness checking (skip duplicate/stale results)
        if timestamp <= self._last_processed_timestamp:
            if publish:
                SmartDashboard.putBoolean("Vision/ValidPose", False)
                SmartDashboard.putString("Vision/RejectReason", "Stale result")
            return
        self._last_processed_timestamp = timestamp

        if abs(vision_pose_3d.z) > Z_AXIS_THRESHOLD:
            if publish:
                SmartDashboard.putBoolean("Vision/ValidPose", False)
                SmartDashboard.putString(
                    "Vision/RejectReason", f"Invalid Z: {vision_pose_3d.z:.3f}m"
                )
            return

        roll_deg = abs(vision_pose_3d.rotation().x)
        pitch_deg = abs(vision_pose_3d.rotation().y)
        if roll_deg > TILT_THRESHOLD_DEG or pitch_deg > TILT_THRESHOLD_DEG:
            if publish:
                SmartDashboard.putBoolean("Vision/ValidPose", False)
                SmartDashboard.putString(
                    "Vision/RejectReason",
                    f"Invalid tilt: roll={roll_deg:.1f}°, pitch={pitch_deg:.1f}°",
                )
            return

        vision_pose_2d = vision_pose_3d.toPose2d()

        if self._last_valid_pose is not None:
            distance_jump = self._last_valid_pose.translation().distance(
                vision_pose_2d.translation()
            )
            if distance_jump > JUMP_THRESHOLD_M:
                if publish:
                    SmartDashboard.putBoolean("Vision/ValidPose", False)
                    SmartDashboard.putString(
                        "Vision/RejectReason", f"Jump: {distance_jump:.3f}m"
                    )
                return

        # Multi-tag vs single-tag discrepancy check
        if estimation.single_tag_pose is not None:
            multi_pose = vision_pose_3d
            single_pose = estimation.single_tag_pose.estimatedPose
            x_diff = abs(multi_pose.X() - single_pose.X())
            y_diff = abs(multi_pose.Y() - single_pose.Y())
            if x_diff > 0.5 or y_diff > 0.5:
                if publish:
                    SmartDashboard.putBoolean("Vision/ValidPose", False)
                    SmartDashboard.putString(
                        "Vision/RejectReason",
                        f"Multi/Single discrepancy: dx={x_diff:.3f}m dy={y_diff:.3f}m",
                    )
                return

        # Calculate dynamic standard deviations based on measurement quality
        std_dev_x, std_dev_y, std_dev_rot = self._calculate_measurement_std_devs(
            result, best_target
        )

        # Add vision measurement to the drivetrain's pose estimator
        self._drivetrain.add_vision_measurement(
            vision_pose_2d, timestamp, (std_dev_x, std_dev_y, std_dev_rot)
        )

        # Update state tracking (always)
        self._last_valid_pose = vision_pose_2d
        self._last_valid_update_time = Timer.getFPGATimestamp()

        # Publish diagnostics (throttled)
        if publish:
            SmartDashboard.putBoolean("Vision/ValidPose", True)
            SmartDashboard.putString("Vision/RejectReason", "None")
            SmartDashboard.putNumber("Vision/PoseX", vision_pose_2d.X())
            SmartDashboard.putNumber("Vision/PoseY", vision_pose_2d.Y())
            SmartDashboard.putNumber(
                "Vision/PoseRotation", vision_pose_2d.rotation().degrees()
            )
            SmartDashboard.putNumber("Vision/Latency", result.getLatencyMillis())
            SmartDashboard.putNumber("Vision/StdDevX", std_dev_x)
            SmartDashboard.putNumber("Vision/StdDevY", std_dev_y)
            SmartDashboard.putNumber("Vision/StdDevRot", std_dev_rot)

    def _calculate_measurement_std_devs(self, result, best_target) -> tuple:
        """
        Calculate dynamic standard deviations based on measurement quality.

        Scales confidence based on:
        - Tag count (more tags = more accurate)
        - Distance to target (closer = more accurate)
        - Ambiguity (lower = more confident)

        Args:
            result: PhotonPipelineResult from camera
            best_target: Best PhotonTrackedTarget from result

        Returns:
            Tuple of (std_dev_x, std_dev_y, std_dev_rotation) in (m, m, deg)
        """
        # Base standard deviations
        base_std_dev = (0.1, 0.1, 2.0)

        # Get tag count (multi-tag = more accurate)
        tag_count = len(result.getTargets())

        # Get distance to best target
        if best_target is not None:
            distance = best_target.getBestCameraToTarget().translation().norm()
        else:
            distance = 5.0  # Default estimate if no best target

        # Get ambiguity
        if best_target is not None:
            ambiguity = best_target.getPoseAmbiguity()
        else:
            ambiguity = 0.0

        # Scale factors:
        # - More tags = lower std dev (more accurate)
        # - Closer = lower std dev (more accurate)
        # - Lower ambiguity = lower std dev (more confident)

        tag_count_factor = 1.0 / (
            1.0 + 0.2 * (tag_count - 1)
        )  # 1.0 for 1 tag, 0.83 for 2 tags, 0.71 for 3 tags
        distance_factor = 1.0 + 0.1 * distance  # Further = more uncertainty
        ambiguity_factor = 1.0 + ambiguity * 2.0  # Higher ambiguity = more uncertainty

        combined_factor = tag_count_factor * distance_factor * ambiguity_factor

        # Apply scaling
        std_dev_x = base_std_dev[0] * combined_factor
        std_dev_y = base_std_dev[1] * combined_factor
        std_dev_rot = base_std_dev[2] * combined_factor

        return (std_dev_x, std_dev_y, std_dev_rot)

    def get_target_pose(self, target_id: int) -> Optional[Pose2d]:
        """
        Get the field-relative pose of a specific AprilTag.

        Args:
            target_id: The ID of the AprilTag to query

        Returns:
            Pose2d of the tag in field coordinates, or None if tag doesn't exist
        """
        tag = self._field_layout.getTagPose(target_id)

        if tag is None:
            return None

        # Convert Pose3d to Pose2d
        return tag.toPose2d()

    def has_targets(self) -> bool:
        """
        Check if the camera currently sees any AprilTags.

        Uses the cached result from the current periodic() cycle for
        frame consistency with pose estimation.

        Returns:
            True if any targets are visible, False otherwise
        """
        if self._cached_result is None:
            return False
        return self._cached_result.hasTargets()

    def get_best_target_id(self) -> Optional[int]:
        """
        Get the ID of the best (closest/most confident) target.

        Uses the cached result from the current periodic() cycle for
        frame consistency with pose estimation.

        Returns:
            The AprilTag ID of the best target, or None if no targets visible
        """
        if self._cached_result is None:
            return None

        if not self._cached_result.hasTargets():
            return None

        best_target = self._cached_result.getBestTarget()
        if best_target is None:
            return None

        return best_target.getFiducialId()

    def get_visible_tag_ids(self) -> list[int]:
        """
        Get fiducial IDs for all currently visible AprilTags.

        Uses the cached result from the current periodic() cycle for
        frame consistency with pose estimation.

        Returns:
            List of AprilTag IDs currently detected by the camera
        """
        if self._cached_result is None:
            return []
        if not self._cached_result.hasTargets():
            return []
        return [target.getFiducialId() for target in self._cached_result.getTargets()]

    def get_time_since_last_valid_update(self) -> float:
        """
        Get the time since the last valid vision pose update.

        Returns:
            Time in seconds since last valid update
        """
        return Timer.getFPGATimestamp() - self._last_valid_update_time

    @property
    def target_distance(self) -> float:
        """Distance to tracked target in meters (0.0 = no target)."""
        return self._target_distance

    @target_distance.setter
    def target_distance(self, value: float) -> None:
        self._target_distance = value

    @property
    def heading_error(self) -> float:
        """Absolute heading error to target in degrees (180.0 = no target)."""
        return self._heading_error

    @heading_error.setter
    def heading_error(self, value: float) -> None:
        self._heading_error = value
