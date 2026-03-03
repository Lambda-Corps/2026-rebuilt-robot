"""Pose Estimator Utilities

Shared utilities for pose estimation from PhotonVision data.
Used by both VisionSubsystem (robot) and VisionClient (diagnostic app).

Provides:
- Multi-tag → single-tag fallback strategy
- Estimation mode tracking for logging
- Tag ID tracking to detect mode changes
"""

from typing import Optional, Tuple, Any
from dataclasses import dataclass


@dataclass
class EstimationResult:
    """Result of pose estimation with metadata."""

    estimated_pose: Any
    is_multi_tag: bool
    tag_ids: Tuple[int, ...]
    targets_used: list
    single_tag_pose: Any = None


class EstimationModeTracker:
    """
    Tracks estimation mode changes to enable conditional logging.

    Only logs when mode (multi-tag vs single-tag) or tag combination changes,
    preventing console spam while still providing visibility into state changes.
    """

    def __init__(self) -> None:
        self._last_is_multi_tag: Optional[bool] = None
        self._last_tag_ids: Optional[Tuple[int, ...]] = None

    def check_mode_change(
        self, is_multi_tag: bool, tag_ids: Tuple[int, ...]
    ) -> Tuple[bool, Optional[str]]:
        """
        Check if mode or tag combination changed.

        Args:
            is_multi_tag: True if using multi-tag estimation
            tag_ids: Sorted tuple of tag IDs being used

        Returns:
            Tuple of (changed: bool, message: Optional[str])
            - changed: True if mode or tags changed since last call
            - message: Description of change, or None if no change
        """
        mode_changed = is_multi_tag != self._last_is_multi_tag
        tags_changed = tag_ids != self._last_tag_ids

        if mode_changed or tags_changed:
            old_mode = "Multi-tag" if self._last_is_multi_tag else "Single-tag"
            new_mode = "Multi-tag" if is_multi_tag else "Single-tag"

            self._last_is_multi_tag = is_multi_tag
            self._last_tag_ids = tag_ids

            if mode_changed:
                message = (
                    f"Mode changed: {old_mode} → {new_mode} using tags {list(tag_ids)}"
                )
            else:
                message = f"Tags changed: {list(tag_ids)} ({new_mode})"

            return True, message

        return False, None

    def reset(self) -> None:
        """Reset tracking state."""
        self._last_is_multi_tag = None
        self._last_tag_ids = None


def get_estimated_pose_with_fallback(
    pose_estimator: Any, result: Any
) -> Optional[EstimationResult]:
    """
    Get estimated robot pose using multi-tag → single-tag fallback strategy.

    PhotonLib 2026+ uses method-based strategy selection:
    - estimateCoprocMultiTagPose() = best for multi-tag (coprocessor-side PnP)
    - estimateLowestAmbiguityPose() = fallback for single-tag

    Args:
        pose_estimator: PhotonPoseEstimator instance
        result: PhotonPipelineResult from camera

    Returns:
        EstimationResult with pose and metadata, or None if unavailable
    """
    try:
        multi_tag_pose = pose_estimator.estimateCoprocMultiTagPose(result)
        single_tag_pose = pose_estimator.estimateLowestAmbiguityPose(result)

        estimated_pose = (
            multi_tag_pose if multi_tag_pose is not None else single_tag_pose
        )

        if estimated_pose is None:
            return None

        targets_used = (
            estimated_pose.targetsUsed if hasattr(estimated_pose, "targetsUsed") else []
        )
        is_multi_tag = len(targets_used) > 1
        tag_ids = tuple(sorted(t.getFiducialId() for t in targets_used))

        return EstimationResult(
            estimated_pose=estimated_pose,
            is_multi_tag=is_multi_tag,
            tag_ids=tag_ids,
            targets_used=list(targets_used),
            single_tag_pose=single_tag_pose if is_multi_tag else None,
        )
    except Exception as e:
        print(f"[PoseEstimator] Error getting estimated pose: {e}")
        return None
