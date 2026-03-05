"""ShooterCommand: Distance-interpolated shooter with trigger-gated indexer.

When running:
- Shooter motor runs at duty cycle -0.5 (base speed)
- If a vision target is tracked, speed interpolates linearly by distance:
    1m -> -0.5, 6m -> -1.0
- Indexer motor runs at 0.5 ONLY when a target has been seen (distance > 0),
  robot yaw is within 2 degrees of the target heading,
  AND right trigger is depressed (axis > threshold)

# TODO: This hasn't been calibrated with real robot testing yet, so the distance-to-duty-cycle mapping and indexer conditions may need adjustment.

Distance persists via odometry even when the camera loses sight of the tags.
Reads target distance and heading error directly from VisionSubsystem
(written by VisionTrackTargetPair).
"""

from commands2 import Command
from wpilib import SmartDashboard

from subsystems.shooter_subsystem import ShooterSubsystem
from subsystems.VisionSubsystem import VisionSubsystem

from typing import Callable


class ShooterCommand(Command):
    """Command to run shooter with distance interpolation and trigger-gated indexer."""

    # Shooter duty cycle range (negative = shooting direction)
    SHOOTER_BASE_DUTY_CYCLE = -0.5
    SHOOTER_MAX_DUTY_CYCLE = -1.0

    # Distance range for interpolation (meters)
    DISTANCE_MIN_M = 1.0
    DISTANCE_MAX_M = 6.0

    # Indexer duty cycle when active
    INDEXER_DUTY_CYCLE = 0.5

    # Right trigger threshold for "depressed"
    TRIGGER_THRESHOLD = 0.1

    # Yaw alignment tolerance for indexer (degrees)
    INDEXER_YAW_TOLERANCE_DEG = 2.0

    def __init__(
        self,
        shooter_subsystem: ShooterSubsystem,
        vision_subsystem: VisionSubsystem,
        get_right_trigger: Callable[[], float],
    ) -> None:
        """Initialize shooter command.

        Args:
            shooter_subsystem: The ShooterSubsystem instance
            vision_subsystem: The VisionSubsystem instance (provides target distance and heading error)
            get_right_trigger: Callable returning right trigger axis value (0.0 to 1.0)
        """
        super().__init__()

        self._shooter = shooter_subsystem
        self._vision = vision_subsystem
        self._get_right_trigger = get_right_trigger

        self.addRequirements(shooter_subsystem)

    def initialize(self) -> None:
        """Reset state when command starts."""
        SmartDashboard.putString("Shooter/State", "RUNNING")
        SmartDashboard.putBoolean("Shooter/IndexerActive", False)

    def execute(self) -> None:
        """Run shooter with distance interpolation; conditionally run indexer."""
        distance = self._vision.target_distance
        has_target = distance > 0.0

        # Shooter speed: interpolated when target has been seen, base speed otherwise
        if has_target:
            shooter_speed = self._interpolate_shooter_speed(distance)
        else:
            shooter_speed = self.SHOOTER_BASE_DUTY_CYCLE

        self._shooter.set_shooter_speed(shooter_speed)

        # Indexer: only when aligned to target AND trigger depressed
        heading_error = self._vision.heading_error
        is_aligned = heading_error < self.INDEXER_YAW_TOLERANCE_DEG
        trigger_value = self._get_right_trigger()
        is_trigger_depressed = trigger_value > self.TRIGGER_THRESHOLD

        if has_target and is_aligned and is_trigger_depressed:
            self._shooter.set_indexer_speed(self.INDEXER_DUTY_CYCLE)
            SmartDashboard.putBoolean("Shooter/IndexerActive", True)
        else:
            self._shooter.set_indexer_speed(0.0)
            SmartDashboard.putBoolean("Shooter/IndexerActive", False)

        # Diagnostics for indexer gating conditions
        SmartDashboard.putNumber("Shooter/ComputedDutyCycle", shooter_speed)
        SmartDashboard.putBoolean("Shooter/HasTarget", has_target)
        SmartDashboard.putNumber("Shooter/HeadingError", heading_error)

    def _interpolate_shooter_speed(self, distance: float) -> float:
        """Linearly interpolate shooter duty cycle based on target distance.

        1m -> -0.5, 6m -> -1.0, clamped at boundaries.
        """
        clamped = max(self.DISTANCE_MIN_M, min(self.DISTANCE_MAX_M, distance))
        t = (clamped - self.DISTANCE_MIN_M) / (self.DISTANCE_MAX_M - self.DISTANCE_MIN_M)
        return self.SHOOTER_BASE_DUTY_CYCLE + t * (
            self.SHOOTER_MAX_DUTY_CYCLE - self.SHOOTER_BASE_DUTY_CYCLE
        )

    def end(self, interrupted: bool) -> None:
        """Stop all motors when command ends."""
        self._shooter.stop()
        SmartDashboard.putString("Shooter/State", "IDLE")
        SmartDashboard.putBoolean("Shooter/IndexerActive", False)

    def isFinished(self) -> bool:
        """Never finishes -- runs until toggled off."""
        return False
