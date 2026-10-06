"""PhotonVision camera definitions.

Each entry in CAMERAS is one PhotonVision camera that VisionSubsystem will
read and fuse into the drivetrain pose estimate. To add, remove, or re-mount
a camera, only this file needs to change.

- name:             Must exactly match the camera name shown in the
                    PhotonVision web UI (Cameras tab).
- robot_to_camera:  Transform from robot center (on the floor) to the camera
                    lens, in WPILib coordinates: +X forward, +Y left, +Z up
                    (meters). Rotation3d(roll, pitch, yaw) in radians; a
                    camera tilted UP has a NEGATIVE pitch, and a camera facing
                    backward has a yaw of 180 degrees.

Cameras listed earlier take priority when a single answer is needed
(e.g. get_best_target_id).
"""

import math
from dataclasses import dataclass

from wpimath.geometry import Rotation3d, Transform3d, Translation3d


@dataclass(frozen=True)
class CameraConfig:
    name: str
    robot_to_camera: Transform3d


FRONT_CAMERA = CameraConfig(
    name="OV9281",
    robot_to_camera=Transform3d(
        Translation3d(0.3, 0.0, 0.5),
        Rotation3d(0.0, math.radians(-15), 0.0),
    ),
)

# TODO: Set name to match PhotonVision and measure the real mounting position.
# Currently mirrors the front camera on the rear of the robot, facing backward.
REAR_CAMERA = CameraConfig(
    name="OV9281_Rear",
    robot_to_camera=Transform3d(
        Translation3d(-0.3, 0.0, 0.5),
        Rotation3d(0.0, math.radians(-15), math.radians(180)),
    ),
)

CAMERAS: tuple[CameraConfig, ...] = (FRONT_CAMERA, REAR_CAMERA)
