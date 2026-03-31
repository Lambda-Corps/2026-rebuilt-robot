import math

import commands2
from wpilib import DriverStation, SmartDashboard, Timer
from wpilib.shuffleboard import Shuffleboard
from wpimath.geometry import Rotation3d, Transform3d, Translation3d

from wpilib import RobotBase

from constants import (
    POSE_AMBIGUITY_THRESHOLD,
    CAMERA_NAME,
    ROBOT_TO_CAMERA,
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
        tf = t.getBestCameraToTarget()
        trans = tf.translation()
        total_dist += math.sqrt(trans.x ** 2 + trans.y ** 2 + trans.z ** 2)
    avg_dist = total_dist / len(targets)

    if len(targets) >= 2:
        xy = 0.1 + avg_dist ** 2 * 0.05
    else:
        xy = 0.5 + avg_dist ** 2 * 0.1

    # Completely reject the vision heading by providing a massive std dev,
    # forcing the estimator to 100% trust the Pigeon 2 gyro!
    return (xy, xy, 9999999.0)


class VisionSubsystem(commands2.Subsystem):
    CAMERA_NAME = CAMERA_NAME
    POSE_AMBIGUITY_THRESHOLD = POSE_AMBIGUITY_THRESHOLD
    ROBOT_TO_CAMERA = ROBOT_TO_CAMERA
    _SIM_UPDATE_PERIOD = _SIM_UPDATE_PERIOD

    def __init__(self, drivetrain) -> None:
        super().__init__()
        self._drivetrain = drivetrain
        self._last_valid_update_time = 0.0
        self._consecutive_camera_failures = 0

        if PhotonCamera is None or AprilTagFieldLayout is None:
            self._camera = None
            self._pose_estimator = None
            self._field_layout = None
            return

        self._field_layout = AprilTagFieldLayout.loadField(
            AprilTagField.k2026RebuiltWelded
        )
        self._camera = PhotonCamera(self.CAMERA_NAME)
        self._pose_estimator = PhotonPoseEstimator(
            self._field_layout,
            self.ROBOT_TO_CAMERA,
        )

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

        props = SimCameraProperties.PERFECT_90DEG()
        props.setFPS(20)
        props.setAvgLatency(0.035)

        self._camera_sim = PhotonCameraSim(self._camera, props)
        self._vision_sim.addCamera(self._camera_sim, self.ROBOT_TO_CAMERA)

        def _sim_update():
            pose = self._drivetrain.get_state().pose
            self._vision_sim.update(pose)

        self._sim_notifier = Notifier(_sim_update)
        self._sim_notifier.startPeriodic(self._SIM_UPDATE_PERIOD)

    def periodic(self) -> None:
        if self._camera is None or self._pose_estimator is None:
            return

        try:
            result = self._camera.getLatestResult()
            self._consecutive_camera_failures = 0
        except Exception:
            self._consecutive_camera_failures += 1
            SmartDashboard.putBoolean("Vision/CameraConnected", False)
            return

        SmartDashboard.putBoolean("Vision/CameraConnected", True)
        SmartDashboard.putBoolean("Vision/HasTargets", result.hasTargets())

        if not result.hasTargets():
            return

        targets = result.getTargets()
        multi_tag = len(targets) >= 2

        if not multi_tag:
            # Single-tag: reject ambiguous poses (two equally-valid mirror solutions)
            best = result.getBestTarget()
            if best is not None and best.getPoseAmbiguity() > self.POSE_AMBIGUITY_THRESHOLD:
                return

        # Multi-tag PnP is unambiguous; fall back to best single-tag only if needed
        estimated = self._pose_estimator.estimateCoprocMultiTagPose(result)
        if estimated is None:
            estimated = self._pose_estimator.estimateLowestAmbiguityPose(result)
        if estimated is None:
            return

        std_devs = _compute_std_devs(estimated)
        self._drivetrain.add_vision_measurement(
            estimated.estimatedPose.toPose2d(),
            estimated.timestampSeconds,
            std_devs,
        )
        self._last_valid_update_time = Timer.getFPGATimestamp()
        SmartDashboard.putNumber(
            "Vision/TimeSinceUpdate", self.get_time_since_last_valid_update()
        )

    def get_target_pose(self, tag_id: int):
        """Return Pose3d of the given AprilTag from field layout, or None."""
        if self._field_layout is None:
            return None
        try:
            return self._field_layout.getTagPose(tag_id)
        except Exception:
            return None

    def has_targets(self) -> bool:
        """Return True if the camera currently sees any targets."""
        if self._camera is None:
            return False
        try:
            result = self._camera.getLatestResult()
            return result.hasTargets()
        except Exception:
            return False

    def get_best_target_id(self):
        """Return fiducial ID of best target, or None."""
        if self._camera is None:
            return None
        try:
            result = self._camera.getLatestResult()
            if not result.hasTargets():
                return None
            best = result.getBestTarget()
            return best.getFiducialId()
        except Exception:
            return None

    def get_visible_tag_ids(self) -> list:
        """Return list of fiducial IDs for all currently visible tags."""
        if self._camera is None:
            return []
        try:
            result = self._camera.getLatestResult()
            if not result.hasTargets():
                return []
            return [t.getFiducialId() for t in result.getTargets()]
        except Exception:
            return []

    def get_time_since_last_valid_update(self) -> float:
        """Return seconds since the last accepted vision measurement."""
        return Timer.getFPGATimestamp() - self._last_valid_update_time

    def seed_drivetrain_pose(self) -> bool:
        """Seed the drivetrain's pose (including yaw) from the best vision reading.
        Returns True if successful, False if no targets are currently visible.
        """
        if self._camera is None or self._pose_estimator is None:
            return False

        try:
            result = self._camera.getLatestResult()
            if not result.hasTargets():
                return False

            estimated = self._pose_estimator.estimateCoprocMultiTagPose(result)
            if estimated is None:
                estimated = self._pose_estimator.estimateLowestAmbiguityPose(result)

            if estimated is None:
                return False

            self._drivetrain.reset_pose(estimated.estimatedPose.toPose2d())
            return True
        except Exception:
            return False
