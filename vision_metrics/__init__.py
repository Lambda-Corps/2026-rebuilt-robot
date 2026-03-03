"""
Vision Metrics Module

Pure calculation functions for vision metrics that can be reused by both
VisionApp (UI) and robot.py (SmartDashboard/SignalLogger logging).

Contains NO UI-specific code. All calculations return data structures suitable
for any display or logging context.
"""

from vision_metrics.constants import (
    CAMERA_NAME,
    ROBOT_TO_CAM_X,
    ROBOT_TO_CAM_Y,
    ROBOT_TO_CAM_Z,
    AMBIGUITY_THRESHOLD,
    Z_AXIS_THRESHOLD,
    TILT_THRESHOLD_DEG,
    JUMP_THRESHOLD_M,
    CONFIDENCE_THRESHOLD_HIGH,
    CONFIDENCE_THRESHOLD_LOW,
    USE_SIGNALLOGGER_FOR_VISION,
)

from vision_metrics.tag_detection_metrics import (
    get_tag_detection_info,
    get_all_tags_info,
    TagDetectionInfo,
)

from vision_metrics.pose_metrics import (
    get_robot_pose_info,
    RobotPoseInfo,
)

from vision_metrics.pose_estimator import (
    get_estimated_pose_with_fallback,
    EstimationResult,
    EstimationModeTracker,
)

__all__ = [
    # Constants
    "CAMERA_NAME",
    "ROBOT_TO_CAM_X",
    "ROBOT_TO_CAM_Y",
    "ROBOT_TO_CAM_Z",
    "AMBIGUITY_THRESHOLD",
    "Z_AXIS_THRESHOLD",
    "TILT_THRESHOLD_DEG",
    "JUMP_THRESHOLD_M",
    "CONFIDENCE_THRESHOLD_HIGH",
    "CONFIDENCE_THRESHOLD_LOW",
    "USE_SIGNALLOGGER_FOR_VISION",
    # Tag detection metrics
    "get_tag_detection_info",
    "get_all_tags_info",
    "TagDetectionInfo",
    # Pose metrics
    "get_robot_pose_info",
    "RobotPoseInfo",
    # Pose estimator utilities
    "get_estimated_pose_with_fallback",
    "EstimationResult",
    "EstimationModeTracker",
]
