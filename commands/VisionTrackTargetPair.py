"""VisionTrackTargetPair: Smooth rotation tracking for configurable AprilTag pairs.

When held, robot tracks alliance-specific tag pairs with smooth acceleration
and 3-frame smoothing to tolerate flicker. Only controls rotation; allows driver
to translate freely. Relinquishes control when no targets visible.

Key Features:
- Alliance-based target pairs (Red vs Blue)
- 3-frame moving average smoothing for target flicker tolerance
- Trapezoidal motion profile (SlewRateLimiter) for smooth accel/decel
- Rotation-only control (driver maintains translation)
- Graceful handoff when no tags visible (returns control to driver)
- Maintains last target briefly during flicker (250ms timeout)
"""

from commands2 import Command
from wpilib import SmartDashboard, Timer, DriverStation
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds

from typing import Optional
from collections import deque
import math

from vision_metrics.constants import (
    TAG_PAIR_OFFSET_BEHIND_METERS,
    TAG_PAIR_OFFSET_RIGHT_METERS,
    RED_ALLIANCE_PAIRS,
    BLUE_ALLIANCE_PAIRS,
    TRACK_ROTATION_KP,
    TRACK_ROTATION_KI,
    TRACK_ROTATION_KD,
    TRACK_ROTATION_TOLERANCE_DEG,
    TRACK_CONSECUTIVE_LOOPS_REQUIRED,
    TRACK_SMOOTHING_FRAMES,
    TRACK_MAX_ROTATION_SPEED,
    TRACK_MAX_ROTATION_ACCEL,
    TRACK_ROTATION_DEADBAND_DEG,
    TRACK_TARGET_PERSISTENCE_TIMEOUT,
)


class VisionTrackTargetPair(Command):
    """
    Command to smoothly rotate robot toward alliance-specific AprilTag pairs.

    Uses 3-frame smoothing and trapezoidal motion profiling for smooth tracking.
    Relinquishes rotation control when no tags are visible.
    """


    def __init__(
        self,
        drivetrain,
        vision_subsystem,
        get_translation_input=None,
        get_drive_request=None,
        get_rotation_input=None,
    ):
        """
        Initialize vision tracking command.

        Args:
            drivetrain: The CommandSwerveDrivetrain instance
            vision_subsystem: The VisionSubsystem instance
            get_translation_input: Optional callable that returns (vx, vy) translation tuple
                                   from joystick (WPILib convention: vx=forward, vy=strafe).
                                   If None, uses zero translation.
            get_drive_request: Optional callable that returns a configured SwerveRequest
                              (FieldCentric or RobotCentric) appropriate for current mode.
                              If None, translation is zeroed.
            get_rotation_input: Optional callable that returns rotation rate (rad/s)
                               from joystick. Used when no vision target is available
                               so driver retains rotation control. If None, rotation
                               is zeroed when no target.
        """
        super().__init__()

        self._drivetrain = drivetrain
        self._vision = vision_subsystem
        self._get_translation_input = get_translation_input
        self._get_drive_request = get_drive_request
        self._get_rotation_input = get_rotation_input

        # PID controller for rotation
        self._pid_rotation = PIDController(
            TRACK_ROTATION_KP, TRACK_ROTATION_KI, TRACK_ROTATION_KD
        )
        self._pid_rotation.enableContinuousInput(-180, 180)

        # Trapezoidal motion profile (slew rate limiter)
        self._rotation_slew_limiter = SlewRateLimiter(TRACK_MAX_ROTATION_ACCEL)

        # 3-frame smoothing buffer for target rotation
        self._target_rotation_buffer: deque[float] = deque(maxlen=TRACK_SMOOTHING_FRAMES)

        # State tracking
        self._target_pair: Optional[tuple[int, int]] = None
        self._locked_pair: Optional[tuple[int, int]] = (
            None  # Lock onto first selected pair
        )
        self._smoothed_target_rotation: Optional[Rotation2d] = None
        self._last_valid_target_time: float = 0.0
        self._consecutive_loops_in_tolerance = 0
        self._is_active: bool = False
        self._last_target_offset: Optional[tuple[float, float]] = None

        # Require both subsystems
        self.addRequirements(drivetrain, vision_subsystem)

    def initialize(self):
        """Called when command starts."""
        self._target_pair = None
        self._locked_pair = None  # Reset locked pair
        self._smoothed_target_rotation = None
        self._last_valid_target_time = 0.0
        self._consecutive_loops_in_tolerance = 0
        self._is_active = False
        self._last_target_offset = None

        # Clear smoothing buffer
        self._target_rotation_buffer.clear()

        # Reset controllers
        self._pid_rotation.reset()
        self._rotation_slew_limiter.reset(0.0)

        SmartDashboard.putString("VisionTrack/State", "SEARCHING")
        SmartDashboard.putString("VisionTrack/TargetPair", "None")
        SmartDashboard.putNumber("VisionTrack/ToleranceLoops", 0)
        SmartDashboard.putBoolean("VisionTrack/Active", False)
        SmartDashboard.putNumber("VisionTrack/TargetDistance", 0.0)
        SmartDashboard.putNumber("VisionTrack/HeadingError", 180.0)

    def execute(self):
        """Called repeatedly while command is scheduled."""
        current_time = Timer.getFPGATimestamp()
        current_pose = self._drivetrain.get_state().pose
        current_rotation = current_pose.rotation().degrees()

        # Get alliance-specific target pairs
        target_pairs = self._get_target_pairs_for_alliance()

        # Find target rotation (with smoothing)
        target_rotation = self._update_target_rotation(
            current_pose, target_pairs, current_time
        )

        # Compute distance and heading error to last known target offset
        if self._last_target_offset is not None:
            dx = self._last_target_offset[0] - current_pose.X()
            dy = self._last_target_offset[1] - current_pose.Y()
            SmartDashboard.putNumber(
                "VisionTrack/TargetDistance", math.sqrt(dx * dx + dy * dy)
            )
            # Heading error between robot facing and target direction
            target_heading_deg = math.degrees(math.atan2(dy, dx))
            heading_error = target_heading_deg - current_rotation
            while heading_error > 180:
                heading_error -= 360
            while heading_error < -180:
                heading_error += 360
            SmartDashboard.putNumber(
                "VisionTrack/HeadingError", abs(heading_error)
            )
        else:
            SmartDashboard.putNumber("VisionTrack/TargetDistance", 0.0)
            SmartDashboard.putNumber("VisionTrack/HeadingError", 180.0)

        # Determine rotation: vision-controlled if tracking, driver-controlled otherwise
        if target_rotation is None:
            # No valid target - use driver rotation input
            self._is_active = False
            SmartDashboard.putBoolean("VisionTrack/Active", False)
            SmartDashboard.putString("VisionTrack/State", "NO_TARGET")

            if self._get_rotation_input is not None:
                omega = self._get_rotation_input()
            else:
                omega = 0.0

            SmartDashboard.putNumber("VisionTrack/Omega", omega)
        else:
            # Valid target - vision controls rotation
            self._is_active = True
            SmartDashboard.putBoolean("VisionTrack/Active", True)

            # Calculate error for telemetry (normalize to -180..180)
            error_rotation = target_rotation.degrees() - current_rotation
            while error_rotation > 180:
                error_rotation -= 360
            while error_rotation < -180:
                error_rotation += 360

            SmartDashboard.putNumber("VisionTrack/ErrorRotation", error_rotation)
            SmartDashboard.putNumber(
                "VisionTrack/TargetRotation", target_rotation.degrees()
            )

            # Calculate raw PID output using measurement and setpoint directly
            # so enableContinuousInput handles angle wrapping correctly
            if abs(error_rotation) < TRACK_ROTATION_DEADBAND_DEG:
                raw_omega = 0.0
            else:
                raw_omega = self._pid_rotation.calculate(
                    current_rotation, target_rotation.degrees()
                )

            # Apply trapezoidal motion profile (smooth acceleration/deceleration)
            omega = self._rotation_slew_limiter.calculate(raw_omega)

            # Clamp max rotation speed
            omega = max(-TRACK_MAX_ROTATION_SPEED, min(TRACK_MAX_ROTATION_SPEED, omega))

            SmartDashboard.putNumber("VisionTrack/Omega", omega)
            SmartDashboard.putNumber("Debug/RawOmega", raw_omega)

        # Get translation input from driver (if callback provided)
        if self._get_translation_input is not None:
            trans_x, trans_y = self._get_translation_input()
        else:
            trans_x, trans_y = 0.0, 0.0

        SmartDashboard.putNumber("VisionTrack/TransX", trans_x)
        SmartDashboard.putNumber("VisionTrack/TransY", trans_y)

        if self._get_drive_request is not None:
            request = self._get_drive_request()
            self._drivetrain.set_control(
                request.with_velocity_x(trans_x)
                .with_velocity_y(trans_y)
                .with_rotational_rate(omega)
            )
        else:
            self._drivetrain.set_control(
                self._drivetrain._apply_robot_speeds.with_speeds(
                    ChassisSpeeds(0, 0, omega)
                )
            )

        # Check tolerance (only when vision is actively tracking)
        if self._is_active and target_rotation is not None:
            if abs(error_rotation) < TRACK_ROTATION_TOLERANCE_DEG:
                self._consecutive_loops_in_tolerance += 1
                SmartDashboard.putNumber(
                    "VisionTrack/ToleranceLoops",
                    self._consecutive_loops_in_tolerance,
                )

                if (
                    self._consecutive_loops_in_tolerance
                    >= TRACK_CONSECUTIVE_LOOPS_REQUIRED
                ):
                    SmartDashboard.putString("VisionTrack/State", "LOCKED")
            else:
                self._consecutive_loops_in_tolerance = 0
                SmartDashboard.putString("VisionTrack/State", "TRACKING")

    def _get_target_pairs_for_alliance(self) -> list[tuple[int, int]]:
        """Get target pairs based on current alliance."""
        alliance = DriverStation.getAlliance()

        if alliance == DriverStation.Alliance.kRed:
            return RED_ALLIANCE_PAIRS
        elif alliance == DriverStation.Alliance.kBlue:
            return BLUE_ALLIANCE_PAIRS
        else:
            # Default to red alliance if unknown
            return RED_ALLIANCE_PAIRS

    def _update_target_rotation(
        self,
        robot_pose: Pose2d,
        target_pairs: list[tuple[int, int]],
        current_time: float,
    ) -> Optional[Rotation2d]:
        """
        Update target rotation with 3-frame smoothing and flicker tolerance.

        Returns smoothed target rotation or None if no target available.
        """
        visible_ids = set(self._vision.get_visible_tag_ids())

        # Check if our locked pair is still visible
        if self._locked_pair is not None:
            pair_visible = (
                self._locked_pair[0] in visible_ids
                and self._locked_pair[1] in visible_ids
            )
            if pair_visible:
                # Keep using locked pair
                matching_pairs = [self._locked_pair]
            else:
                # Locked pair no longer visible, clear it
                self._locked_pair = None
                matching_pairs = []
        else:
            matching_pairs = []

        # If no locked pair, find new matching pairs
        if self._locked_pair is None:
            matching_pairs = [
                pair
                for pair in target_pairs
                if pair[0] in visible_ids and pair[1] in visible_ids
            ]

        if matching_pairs:
            # If we don't have a locked pair, select one and lock onto it
            if self._locked_pair is None:
                # Find best pair (lowest ID priority)
                best_pair = min(matching_pairs, key=lambda p: min(p))
                self._locked_pair = best_pair
            else:
                best_pair = self._locked_pair

            pose_a = self._vision.get_target_pose(best_pair[0])
            pose_b = self._vision.get_target_pose(best_pair[1])

            if pose_a is not None and pose_b is not None:
                # Compute offset target point (behind + right of tag pair)
                offset_x, offset_y = self._compute_offset_point(pose_a, pose_b)
                self._last_target_offset = (offset_x, offset_y)
                target_heading = self._heading_to_point(robot_pose, offset_x, offset_y)

                # Update tracking state
                self._target_pair = best_pair
                self._last_valid_target_time = current_time

                # Add to smoothing buffer
                self._target_rotation_buffer.append(target_heading.degrees())

                SmartDashboard.putString(
                    "VisionTrack/TargetPair", f"{best_pair[0]},{best_pair[1]}"
                )
                SmartDashboard.putNumber(
                    "VisionTrack/BufferSize", len(self._target_rotation_buffer)
                )
                SmartDashboard.putNumber("VisionTrack/OffsetX", offset_x)
                SmartDashboard.putNumber("VisionTrack/OffsetY", offset_y)

        # Check if we should maintain last target during brief flicker
        time_since_valid = current_time - self._last_valid_target_time
        has_persistence = (
            self._target_pair is not None
            and time_since_valid < TRACK_TARGET_PERSISTENCE_TIMEOUT
            and len(self._target_rotation_buffer) > 0
        )

        if not matching_pairs and not has_persistence:
            # No target and no persistence - clear everything
            self._target_pair = None
            self._locked_pair = None  # Also clear locked pair
            self._target_rotation_buffer.clear()
            return None

        # Calculate smoothed rotation from buffer (circular average to handle ±180° wrapping)
        if len(self._target_rotation_buffer) > 0:
            avg_sin = sum(math.sin(math.radians(r)) for r in self._target_rotation_buffer) / len(self._target_rotation_buffer)
            avg_cos = sum(math.cos(math.radians(r)) for r in self._target_rotation_buffer) / len(self._target_rotation_buffer)
            avg_rotation = math.degrees(math.atan2(avg_sin, avg_cos))
            return Rotation2d.fromDegrees(avg_rotation)

        return None

    def _heading_to_point(self, robot_pose: Pose2d, x: float, y: float) -> Rotation2d:
        """Calculate heading from robot position to a field point."""
        dx = x - robot_pose.X()
        dy = y - robot_pose.Y()
        return Rotation2d.fromDegrees(math.degrees(math.atan2(dy, dx)))

    def _compute_offset_point(
        self, pose_a: Pose2d, pose_b: Pose2d
    ) -> tuple[float, float]:
        """Compute offset target point relative to the tag pair.

        Returns a field-coordinate point that is BEHIND meters into the tag pair's
        mounting surface (-X local) and RIGHT meters to the right (-Y local).
        """
        mid_x = (pose_a.X() + pose_b.X()) / 2
        mid_y = (pose_a.Y() + pose_b.Y()) / 2

        # Average tag facing directions (safe angle averaging via atan2)
        theta_a = pose_a.rotation().radians()
        theta_b = pose_b.rotation().radians()
        avg_sin = (math.sin(theta_a) + math.sin(theta_b)) / 2
        avg_cos = (math.cos(theta_a) + math.cos(theta_b)) / 2
        theta = math.atan2(avg_sin, avg_cos)

        # "Behind" = tag's -X (into surface)
        # "Right"  = tag's +Y (robot's right when facing the tag front)
        offset_x = (
            mid_x
            - TAG_PAIR_OFFSET_BEHIND_METERS * math.cos(theta)
            - TAG_PAIR_OFFSET_RIGHT_METERS * math.sin(theta)
        )
        offset_y = (
            mid_y
            - TAG_PAIR_OFFSET_BEHIND_METERS * math.sin(theta)
            + TAG_PAIR_OFFSET_RIGHT_METERS * math.cos(theta)
        )

        return (offset_x, offset_y)

    def end(self, interrupted: bool):
        """Called when command ends."""
        reason = "Interrupted" if interrupted else "Finished"
        if self._consecutive_loops_in_tolerance >= TRACK_CONSECUTIVE_LOOPS_REQUIRED:
            reason = "Locked on target"

        SmartDashboard.putString("VisionTrack/EndReason", reason)
        SmartDashboard.putString("VisionTrack/State", "IDLE")
        SmartDashboard.putBoolean("VisionTrack/Active", False)
        SmartDashboard.putNumber("VisionTrack/Omega", 0.0)
        SmartDashboard.putNumber("VisionTrack/TargetDistance", 0.0)
        SmartDashboard.putNumber("VisionTrack/HeadingError", 180.0)

    def isFinished(self) -> bool:
        """Returns False - command runs until interrupted by driver."""
        return False
