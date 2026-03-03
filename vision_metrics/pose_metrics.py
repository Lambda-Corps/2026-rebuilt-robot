"""Pose Metrics

Pure calculation functions for robot pose estimation extracted from PhotonPoseEstimator.

Reusable by both VisionApp (UI) and robot.py (SmartDashboard/SignalLogger logging).
"""

from typing import Optional, Any


class RobotPoseInfo:
    """Data class for robot pose estimation metrics."""

    def __init__(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        heading_deg: float,
        timestamp: float,
        confidence_pct: Optional[float] = None,
    ):
        self.x_m = x_m
        self.y_m = y_m
        self.z_m = z_m
        self.heading_deg = heading_deg
        self.timestamp = timestamp
        self.confidence_pct = confidence_pct

    def to_dict(self) -> dict:
        """Convert to dictionary for logging."""
        return {
            "x_m": self.x_m,
            "y_m": self.y_m,
            "z_m": self.z_m,
            "heading_deg": self.heading_deg,
            "timestamp": self.timestamp,
            "confidence_pct": self.confidence_pct,
        }


def get_robot_pose_info(
    estimated_pose: Any, latency_ms: float
) -> Optional[RobotPoseInfo]:
    """
    Extract robot pose information from PhotonPoseEstimator result.

    Args:
        estimated_pose: EstimatedRobotPose from PhotonPoseEstimator
        latency_ms: Pipeline latency in milliseconds

    Returns:
        RobotPoseInfo with robot pose metrics, or None if unavailable
    """
    try:
        pose_3d = estimated_pose.estimatedPose
        pose_2d = pose_3d.toPose2d()
        timestamp = estimated_pose.timestampSeconds - (latency_ms / 1000.0)

        return RobotPoseInfo(
            x_m=pose_2d.X(),
            y_m=pose_2d.Y(),
            z_m=pose_3d.Z(),
            heading_deg=pose_2d.rotation().degrees(),
            timestamp=timestamp,
        )
    except Exception as e:
        print(f"[PoseMetrics] Error extracting pose: {e}")
        return None
