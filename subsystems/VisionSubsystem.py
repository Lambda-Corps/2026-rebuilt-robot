import math

import commands2
from wpilib import SmartDashboard, Timer

from wpilib import RobotBase

from camera_config import CAMERAS, CameraConfig
from constants import (
    POSE_AMBIGUITY_THRESHOLD,
    VISION_FIELD_MARGIN,
    VISION_MAX_Z_ERROR,
    VISION_RESULT_MAX_AGE,
    _SIM_UPDATE_PERIOD,
)

try:
    from photonlibpy.photonCamera import PhotonCamera
    from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
except Exception:
    PhotonCamera = None  # type: ignore[assignment,misc]
    PhotonPoseEstimator = None  # type: ignore[assignment,misc]

try:
    from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
except Exception:
    AprilTagField = None  # type: ignore[assignment,misc]
    AprilTagFieldLayout = None  # type: ignore[assignment,misc]

def _compute_std_devs(estimated) -> tuple:
    """Scale measurement uncertainty by tag count and average distance."""
    targets = estimated.targetsUsed
    if not targets:
        return (1.0, 1.0, 9999999.0)

    total_dist = 0.0
    for t in targets:
        total_dist += t.getBestCameraToTarget().translation().norm()
    avg_dist = total_dist / len(targets)

    if len(targets) >= 2:
        xy = 0.1 + avg_dist ** 2 * 0.05
    else:
        xy = 0.5 + avg_dist ** 2 * 0.1

    # Completely reject the vision heading by providing a massive std dev,
    # forcing the estimator to 100% trust the Pigeon 2 gyro!
    return (xy, xy, 9999999.0)


def _estimate_pose(pose_estimator, result):
    """Multi-tag PnP is unambiguous; fall back to best single-tag only if needed."""
    estimated = pose_estimator.estimateCoprocMultiTagPose(result)
    if estimated is None:
        estimated = pose_estimator.estimateLowestAmbiguityPose(result)
    return estimated


class VisionCamera:
    """One PhotonVision camera, the pose estimator for its mounting position,
    and its most recent pipeline result (cached once per loop by VisionSubsystem).

    Image processing and tag detection happen on the PhotonVision coprocessor;
    the robot only receives the pipeline results it publishes over NetworkTables.
    """

    def __init__(self, config: CameraConfig, field_layout) -> None:
        self.config = config
        self.name = config.name
        self.camera = PhotonCamera(config.name)
        self.pose_estimator = PhotonPoseEstimator(
            field_layout,
            config.robot_to_camera,
        )
        self.consecutive_failures = 0
        self._latest_result = None
        self._latest_result_time = 0.0

        prefix = f"Vision/{config.name}"
        self.connected_key = f"{prefix}/CameraConnected"
        self.has_targets_key = f"{prefix}/HasTargets"

    def read_unread_results(self):
        """Drain every pipeline result received since the last call, oldest first.
        Must be called exactly once per loop (the queue is cleared on read).
        Returns None if the camera is disconnected or can't be read.
        """
        if not self.camera.isConnected():
            return None
        try:
            results = self.camera.getAllUnreadResults()
        except Exception:
            return None
        if results:
            self._latest_result = results[-1]
            self._latest_result_time = Timer.getFPGATimestamp()
        return results

    def get_current_result(self):
        """Return the newest cached result, or None if there isn't a recent one."""
        if self._latest_result is None:
            return None
        if Timer.getFPGATimestamp() - self._latest_result_time > VISION_RESULT_MAX_AGE:
            return None
        return self._latest_result


class VisionSubsystem(commands2.Subsystem):
    CAMERAS = CAMERAS
    POSE_AMBIGUITY_THRESHOLD = POSE_AMBIGUITY_THRESHOLD
    _SIM_UPDATE_PERIOD = _SIM_UPDATE_PERIOD

    def __init__(self, drivetrain) -> None:
        super().__init__()
        self._drivetrain = drivetrain
        self._last_valid_update_time = 0.0
        self._cameras: list[VisionCamera] = []

        if PhotonCamera is None or AprilTagFieldLayout is None:
            self._field_layout = None
            return

        self._field_layout = AprilTagFieldLayout.loadField(
            AprilTagField.k2026RebuiltWelded
        )
        self._field_length = self._field_layout.getFieldLength()
        self._field_width = self._field_layout.getFieldWidth()
        self._cameras = [
            VisionCamera(config, self._field_layout) for config in self.CAMERAS
        ]

        if RobotBase.isSimulation():
            self._setup_simulation()

    def _setup_simulation(self) -> None:
        try:
            from photonlibpy.simulation.visionSystemSim import VisionSystemSim
            from photonlibpy.simulation.photonCameraSim import PhotonCameraSim
            from photonlibpy.simulation.simCameraProperties import SimCameraProperties
        except Exception:
            return
        from wpilib import Notifier

        self._vision_sim = VisionSystemSim("main")
        self._vision_sim.addAprilTags(self._field_layout)

        self._camera_sims = []
        for cam in self._cameras:
            props = SimCameraProperties.PERFECT_90DEG()
            props.setFPS(20)
            props.setAvgLatency(0.035)

            camera_sim = PhotonCameraSim(cam.camera, props)
            self._vision_sim.addCamera(camera_sim, cam.config.robot_to_camera)
            self._camera_sims.append(camera_sim)

        def _sim_update():
            pose = self._drivetrain.get_state().pose
            self._vision_sim.update(pose)

        self._sim_notifier = Notifier(_sim_update)
        self._sim_notifier.startPeriodic(self._SIM_UPDATE_PERIOD)

    def periodic(self) -> None:
        if not self._cameras:
            SmartDashboard.putBoolean("Vision/CameraConnected", False)
            SmartDashboard.putBoolean("Vision/HasTargets", False)
            return

        all_connected = True
        any_targets = False
        for cam in self._cameras:
            connected, has_targets = self._process_camera(cam)
            all_connected = all_connected and connected
            any_targets = any_targets or has_targets

        # Aggregate keys: CameraConnected is only True when EVERY camera is up,
        # so the driver dashboard flags a single dead camera.
        SmartDashboard.putBoolean("Vision/CameraConnected", all_connected)
        SmartDashboard.putBoolean("Vision/HasTargets", any_targets)
        SmartDashboard.putNumber(
            "Vision/TimeSinceUpdate", self.get_time_since_last_valid_update()
        )

    def _process_camera(self, cam: VisionCamera) -> tuple[bool, bool]:
        """Feed every unread result from one camera to the drivetrain.
        Returns (connected, has_targets) for dashboard reporting.
        """
        results = cam.read_unread_results()
        if results is None:
            cam.consecutive_failures += 1
            SmartDashboard.putBoolean(cam.connected_key, False)
            SmartDashboard.putBoolean(cam.has_targets_key, False)
            return (False, False)

        cam.consecutive_failures = 0
        for result in results:
            self._add_measurement(cam, result)

        current = cam.get_current_result()
        has_targets = current is not None and current.hasTargets()
        SmartDashboard.putBoolean(cam.connected_key, True)
        SmartDashboard.putBoolean(cam.has_targets_key, has_targets)
        return (True, has_targets)

    def _add_measurement(self, cam: VisionCamera, result) -> None:
        """Estimate the robot pose from one result and, if it passes the
        filters, add it to the drivetrain's pose estimator."""
        if not result.hasTargets():
            return

        if len(result.getTargets()) < 2:
            # Single-tag: reject ambiguous poses (two equally-valid mirror solutions)
            best = result.getBestTarget()
            if best is not None and best.getPoseAmbiguity() > self.POSE_AMBIGUITY_THRESHOLD:
                return

        estimated = _estimate_pose(cam.pose_estimator, result)
        if estimated is None or not self._is_pose_plausible(estimated.estimatedPose):
            return

        self._drivetrain.add_vision_measurement(
            estimated.estimatedPose.toPose2d(),
            estimated.timestampSeconds,
            _compute_std_devs(estimated),
        )
        self._last_valid_update_time = Timer.getFPGATimestamp()

    def _is_pose_plausible(self, pose) -> bool:
        """Reject poses off the field or off the floor (bad PnP solutions)."""
        return (
            -VISION_FIELD_MARGIN <= pose.X() <= self._field_length + VISION_FIELD_MARGIN
            and -VISION_FIELD_MARGIN <= pose.Y() <= self._field_width + VISION_FIELD_MARGIN
            and abs(pose.Z()) <= VISION_MAX_Z_ERROR
        )

    def _current_results_with_targets(self) -> list:
        """Return (camera, result) pairs for every camera currently seeing targets,
        in CAMERAS priority order. Uses the results cached by periodic()."""
        pairs = []
        for cam in self._cameras:
            result = cam.get_current_result()
            if result is not None and result.hasTargets():
                pairs.append((cam, result))
        return pairs

    def get_target_pose(self, tag_id: int):
        """Return Pose3d of the given AprilTag from field layout, or None."""
        if self._field_layout is None:
            return None
        try:
            return self._field_layout.getTagPose(tag_id)
        except Exception:
            return None

    def has_targets(self) -> bool:
        """Return True if any camera currently sees any targets."""
        return len(self._current_results_with_targets()) > 0

    def get_best_target_id(self):
        """Return fiducial ID of the best target from the highest-priority camera
        that sees one, or None."""
        for _, result in self._current_results_with_targets():
            best = result.getBestTarget()
            if best is not None:
                return best.getFiducialId()
        return None

    def get_visible_tag_ids(self) -> list:
        """Return list of fiducial IDs for all tags visible to any camera."""
        tag_ids = []
        for _, result in self._current_results_with_targets():
            for t in result.getTargets():
                tag_id = t.getFiducialId()
                if tag_id not in tag_ids:
                    tag_ids.append(tag_id)
        return tag_ids

    def get_time_since_last_valid_update(self) -> float:
        """Return seconds since the last accepted vision measurement."""
        return Timer.getFPGATimestamp() - self._last_valid_update_time

    def seed_drivetrain_pose(self) -> bool:
        """Seed the drivetrain's pose (including yaw) from the best vision reading
        across all cameras (lowest estimated uncertainty).
        Returns True if successful, False if no targets are currently visible.
        """
        best_estimate = None
        best_xy_std_dev = math.inf
        for cam, result in self._current_results_with_targets():
            try:
                estimated = _estimate_pose(cam.pose_estimator, result)
            except Exception:
                continue
            if estimated is None or not self._is_pose_plausible(estimated.estimatedPose):
                continue
            xy_std_dev = _compute_std_devs(estimated)[0]
            if xy_std_dev < best_xy_std_dev:
                best_estimate = estimated
                best_xy_std_dev = xy_std_dev

        if best_estimate is None:
            return False

        try:
            self._drivetrain.reset_pose(best_estimate.estimatedPose.toPose2d())
            return True
        except Exception:
            return False
