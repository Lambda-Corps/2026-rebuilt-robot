"""Shared constants for vision processing.

Reusable by both robot.py/VisionSubsystem and vision_app.
UI-specific constants (colors, canvas sizes, etc.) remain in vision_app/constants.py.
"""

# Camera configuration - must match PhotonVision config exactly
CAMERA_NAME = "OV9281"

# Robot-to-camera transform (must match physical camera mounting)
# x=forward, y=left, z=up in meters from robot center
ROBOT_TO_CAM_X = 0.25  # forward from robot center (meters)
ROBOT_TO_CAM_Y = 0.0  # lateral from robot center (meters)
ROBOT_TO_CAM_Z = 0.45  # height above ground (meters)

# Filtering thresholds
AMBIGUITY_THRESHOLD = 0.1

# Pose filtering thresholds (used by VisionSubsystem and diagnostics)
Z_AXIS_THRESHOLD = 0.2  # Max Z deviation in meters (robot should be on ground)
TILT_THRESHOLD_DEG = 10.0  # Max roll/pitch in degrees (robot should be flat)
JUMP_THRESHOLD_M = 1.0  # Max pose jump in meters (prevent teleporting)

# Confidence thresholds for multi-tag quality assessment
CONFIDENCE_THRESHOLD_HIGH = 75.0  # Above this = high confidence (Green)
CONFIDENCE_THRESHOLD_LOW = 50.0  # Below this = low confidence (Deep Orange)

# Tag-pair alignment offset (meters)
# BEHIND = distance into the tag's mounting surface (tag's -X direction)
# RIGHT  = distance to the robot's right when facing the tag (tag's +Y direction)
TAG_PAIR_OFFSET_BEHIND_METERS = 0.5
TAG_PAIR_OFFSET_RIGHT_METERS = 0.2

# Alliance-specific target tag pairs
RED_ALLIANCE_PAIRS = [
    (3, 4),
    (5, 8),
    (9, 10),
    (11, 2),
]

BLUE_ALLIANCE_PAIRS = [
    (19, 20),
    (21, 24),
    (25, 56),
    (18, 27),
]

# VisionTrackTargetPair PID constants
TRACK_ROTATION_KP = 3.75
TRACK_ROTATION_KI = 0.0
TRACK_ROTATION_KD = 0.75

# VisionTrackTargetPair rotation tolerances and limits
TRACK_ROTATION_TOLERANCE_DEG = 1.0
TRACK_CONSECUTIVE_LOOPS_REQUIRED = 5
TRACK_SMOOTHING_FRAMES = 5
TRACK_MAX_ROTATION_SPEED = 10.0  # rad/s
TRACK_MAX_ROTATION_ACCEL = 6.0  # rad/s^2
TRACK_ROTATION_DEADBAND_DEG = 0.05
TRACK_TARGET_PERSISTENCE_TIMEOUT = 0.05  # seconds

# Telemetry settings
USE_SIGNALLOGGER_FOR_VISION = (
    False  # Enable SignalLogger for vision metrics data replay
)
