"""Tag Detection Metrics

Pure calculation functions for extracting tag detection information
from PhotonTrackedTarget objects.

Reusable by both VisionApp (UI) and robot.py (SmartDashboard/SignalLogger logging).
"""

from typing import Optional, Any


class TagDetectionInfo:
    """Data class for tag detection metrics."""

    def __init__(
        self,
        tag_id: int,
        yaw_deg: float,
        pitch_deg: float,
        area_pct: float,
        ambiguity: float,
        distance_m: float,
        timestamp: Optional[float] = None,
    ):
        self.tag_id = tag_id
        self.yaw_deg = yaw_deg
        self.pitch_deg = pitch_deg
        self.area_pct = area_pct
        self.ambiguity = ambiguity
        self.distance_m = distance_m
        self.timestamp = timestamp

    def to_dict(self) -> dict:
        """Convert to dictionary for logging."""
        return {
            "tag_id": self.tag_id,
            "yaw_deg": self.yaw_deg,
            "pitch_deg": self.pitch_deg,
            "area_pct": self.area_pct,
            "ambiguity": self.ambiguity,
            "distance_m": self.distance_m,
            "timestamp": self.timestamp,
        }


def get_tag_detection_info(target: Any) -> Optional[TagDetectionInfo]:
    """
    Extract tag detection information from a PhotonTrackedTarget.

    Args:
        target: PhotonTrackedTarget object from PhotonVision

    Returns:
        TagDetectionInfo with tag detection metrics, or None if unavailable
    """
    try:
        tag_id = target.getFiducialId()
        yaw = target.getYaw()
        pitch = target.getPitch()
        area = target.getArea()
        ambiguity = target.getPoseAmbiguity()

        # Get distance from camera-to-target transform
        try:
            cam_to_tag = target.getBestCameraToTarget()
            distance = cam_to_tag.translation().norm()
        except Exception:
            # If distance unavailable, return None
            return None

        return TagDetectionInfo(
            tag_id=tag_id,
            yaw_deg=yaw,
            pitch_deg=pitch,
            area_pct=area,
            ambiguity=ambiguity,
            distance_m=distance,
        )
    except Exception as e:
        print(f"[TagDetectionMetrics] Error extracting info: {e}")
        return None


def get_all_tags_info(targets: list[Any]) -> list[TagDetectionInfo]:
    """
    Extract tag detection info for all visible targets.

    Args:
        targets: List of PhotonTrackedTarget objects

    Returns:
        List of TagDetectionInfo objects
    """
    info_list = []
    for target in targets:
        info = get_tag_detection_info(target)
        if info is not None:
            info_list.append(info)
    return info_list
