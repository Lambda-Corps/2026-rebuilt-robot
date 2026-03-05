"""
VisionAlignTarget: Hybrid command for autonomous alignment to AprilTags.

This command implements a two-phase approach:
1. PathPlanner pathfinding for long-range approach (>0.5m)
2. Direct PID control for precise final docking (<=0.5m)

The command handles:
- Smooth transition between pathfinding and PID states
- Loss of vision target (relies on pose estimator for up to 1 second)
- Precision tolerance checking for completion
- Safety timeouts
"""

from commands2 import Command
from pathplannerlib.auto import AutoBuilder
from wpilib import SmartDashboard, Timer
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Transform2d, Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
import math

from typing import Optional

from vision_metrics.constants import (
    TRACK_ROTATION_KP,
    TRACK_ROTATION_KI,
    TRACK_ROTATION_KD,
)


class VisionAlignTarget(Command):
    """
    Hybrid command that aligns the robot to an AprilTag target.

    The command uses a state machine:
    - PATHFINDING: Uses PathPlanner when >0.5m from goal
    - DIRECT_PID: Uses direct PID control when <=0.5m from goal
    - FINISHED: Within tolerance for required consecutive loops
    """

    # State machine states
    STATE_PATHFINDING = "PATHFINDING"
    STATE_DIRECT_PID = "DIRECT_PID"
    STATE_FINISHED = "FINISHED"

    # Distance threshold for switching from pathfinding to PID (meters)
    HANDOFF_DISTANCE = 0.5

    # PID constants for direct control
    # These may need tuning based on robot performance
    PID_X_KP = 3.0
    PID_X_KI = 0.0
    PID_X_KD = 0.0

    PID_Y_KP = 3.0
    PID_Y_KI = 0.0
    PID_Y_KD = 0.0

    PID_ROTATION_KP = TRACK_ROTATION_KP
    PID_ROTATION_KI = TRACK_ROTATION_KI
    PID_ROTATION_KD = TRACK_ROTATION_KD

    # Tolerance for completion
    POSITION_TOLERANCE_M = 0.05  # 2 cm
    ROTATION_TOLERANCE_DEG = 0.5  # 1/2 degree
    CONSECUTIVE_LOOPS_REQUIRED = 5  # Must be within tolerance for 10 loops

    # Maximum time without vision before timeout (seconds)
    MAX_TIME_WITHOUT_VISION = 1.0

    # Overall command timeout (seconds)
    MAX_TIMEOUT = 10.0

    # Python constructor. Called once when the command object is created (e.g., in setup_button_bindings()
    def __init__(self, drivetrain, vision_subsystem, target_id: int, offset_transform: Transform2d):
        """
        Initialize the VisionAlignTarget command.

        Args:
            drivetrain: The CommandSwerveDrivetrain instance
            vision_subsystem: The VisionSubsystem instance
            target_id: The AprilTag ID to align to
            offset_transform: Transform from the tag to the desired final pose
                             (e.g., Transform2d(Translation2d(0.5, 0), Rotation2d(0))
                             to stop 0.5m in front of the tag)
        """
        super().__init__()

        self._drivetrain = drivetrain
        self._vision = vision_subsystem
        self._target_id = target_id
        self._offset_transform = offset_transform

        # State machine
        self._current_state = self.STATE_PATHFINDING
        self._pathfinding_command: Optional[Command] = None

        # PID controllers for direct control
        self._pid_x = PIDController(self.PID_X_KP, self.PID_X_KI, self.PID_X_KD)
        self._pid_y = PIDController(self.PID_Y_KP, self.PID_Y_KI, self.PID_Y_KD)
        self._pid_rotation = PIDController(
            self.PID_ROTATION_KP,
            self.PID_ROTATION_KI,
            self.PID_ROTATION_KD
        )

        # Configure rotation PID for continuous input (-180 to 180 degrees)
        self._pid_rotation.enableContinuousInput(-180, 180)

        # Tolerance tracking
        self._consecutive_loops_in_tolerance = 0

        # Timing
        self._start_time = 0.0
        self._last_vision_update_time = 0.0

        # Goal pose (calculated in initialize)
        self._goal_pose: Optional[Pose2d] = None

        # Require both subsystems
        self.addRequirements(drivetrain, vision_subsystem)

    # Commands2 lifecycle method. Called every time the command starts running (e.g., each time the button is press Commands2 lifecycle method. Called every time the command starts running (e.g., each time the button is pressed).ed).
    def initialize(self):
        """Called when the command is initially scheduled."""
        self._start_time = Timer.getFPGATimestamp()
        self._last_vision_update_time = self._start_time
        self._consecutive_loops_in_tolerance = 0
        self._current_state = self.STATE_PATHFINDING
        self._pathfinding_command = None

        # Calculate goal pose from target
        target_pose = self._vision.get_target_pose(self._target_id)

        if target_pose is None:
            SmartDashboard.putString("VisionAlign/Error", f"Target {self._target_id} not in field layout")
            self._goal_pose = None
            return

        # Apply offset transform to get desired final pose
        self._goal_pose = target_pose + self._offset_transform

        SmartDashboard.putString("VisionAlign/State", self._current_state)
        SmartDashboard.putNumber("VisionAlign/GoalRotation", self._goal_pose.rotation().degrees())

        # Reset PID controllers
        self._pid_x.reset()
        self._pid_y.reset()
        self._pid_rotation.reset()

    def execute(self):
        """Called repeatedly while the command is scheduled."""
        if self._goal_pose is None:
            # Can't execute without a valid goal
            return

        # Update vision timing
        if self._vision.has_targets():
            self._last_vision_update_time = Timer.getFPGATimestamp()

        # Get current robot pose
        current_pose = self._drivetrain.get_state().pose

        # Calculate distance to goal
        distance_to_goal = self._calculate_distance_to_goal(current_pose)
        SmartDashboard.putNumber("VisionAlign/DistanceToGoal", distance_to_goal)

        # State machine logic
        if self._current_state == self.STATE_PATHFINDING:
            self._execute_pathfinding(current_pose, distance_to_goal)
        elif self._current_state == self.STATE_DIRECT_PID:
            self._execute_direct_pid(current_pose)

        # Check if we're within tolerance
        if self._is_at_goal(current_pose):
            self._consecutive_loops_in_tolerance += 1
            SmartDashboard.putNumber("VisionAlign/ToleranceLoops", self._consecutive_loops_in_tolerance)

            if self._consecutive_loops_in_tolerance >= self.CONSECUTIVE_LOOPS_REQUIRED:
                self._current_state = self.STATE_FINISHED
                SmartDashboard.putString("VisionAlign/State", self._current_state)
        else:
            self._consecutive_loops_in_tolerance = 0

    def _execute_pathfinding(self, current_pose: Pose2d, distance_to_goal: float):
        """
        Execute pathfinding state logic.

        Args:
            current_pose: Current robot pose
            distance_to_goal: Distance to goal in meters
        """
        # Check if we should transition to PID control
        if distance_to_goal <= self.HANDOFF_DISTANCE:
            # Cancel pathfinding command if it exists
            if self._pathfinding_command is not None:
                self._pathfinding_command.cancel()
                self._pathfinding_command = None

            # Transition to PID state
            self._current_state = self.STATE_DIRECT_PID
            SmartDashboard.putString("VisionAlign/State", self._current_state)
            return

        # Start pathfinding if not already running
        if self._pathfinding_command is None:
            self._pathfinding_command = AutoBuilder.pathfindToPose(
                self._goal_pose,
                constraints=None,  # Use default constraints from AutoBuilder config
                goal_end_vel=0.0   # Come to a stop at the goal
            )
            self._pathfinding_command.schedule()

    def _execute_direct_pid(self, current_pose: Pose2d):
        """
        Execute direct PID control state logic.

        Args:
            current_pose: Current robot pose
        """
        # Calculate error in field frame
        dx = self._goal_pose.X() - current_pose.X()
        dy = self._goal_pose.Y() - current_pose.Y()
        d_rotation = self._goal_pose.rotation().degrees() - current_pose.rotation().degrees()

        # Normalize rotation error to [-180, 180]
        while d_rotation > 180:
            d_rotation -= 360
        while d_rotation < -180:
            d_rotation += 360

        # Calculate PID outputs
        vx_field = self._pid_x.calculate(0, -dx)  # Negative because we want to reduce error
        vy_field = self._pid_y.calculate(0, -dy)
        omega = self._pid_rotation.calculate(0, -d_rotation)

        # Convert field-relative to robot-relative speeds
        # Rotate the velocity vector by the negative robot heading
        robot_angle = current_pose.rotation().radians()
        vx_robot = vx_field * math.cos(-robot_angle) - vy_field * math.sin(-robot_angle)
        vy_robot = vx_field * math.sin(-robot_angle) + vy_field * math.cos(-robot_angle)

        # Create ChassisSpeeds and send to drivetrain
        speeds = ChassisSpeeds(vx_robot, vy_robot, omega)
        self._drivetrain.set_control(
            self._drivetrain._apply_robot_speeds.with_speeds(speeds)
        )

        SmartDashboard.putNumber("VisionAlign/PID_VX", vx_robot)
        SmartDashboard.putNumber("VisionAlign/PID_VY", vy_robot)
        SmartDashboard.putNumber("VisionAlign/PID_Omega", omega)

    def _calculate_distance_to_goal(self, current_pose: Pose2d) -> float:
        """
        Calculate Euclidean distance from current pose to goal.

        Args:
            current_pose: Current robot pose

        Returns:
            Distance in meters
        """
        dx = self._goal_pose.X() - current_pose.X()
        dy = self._goal_pose.Y() - current_pose.Y()
        return math.sqrt(dx * dx + dy * dy)

    def _is_at_goal(self, current_pose: Pose2d) -> bool:
        """
        Check if the robot is within tolerance of the goal.

        Args:
            current_pose: Current robot pose

        Returns:
            True if within position and rotation tolerance
        """
        # Check position tolerance
        distance = self._calculate_distance_to_goal(current_pose)
        if distance > self.POSITION_TOLERANCE_M:
            return False

        # Check rotation tolerance
        rotation_error = abs(
            self._goal_pose.rotation().degrees() - current_pose.rotation().degrees()
        )

        # Normalize to [0, 180]
        if rotation_error > 180:
            rotation_error = 360 - rotation_error

        if rotation_error > self.ROTATION_TOLERANCE_DEG:
            return False

        return True

    def isFinished(self) -> bool:
        """
        Check if the command should end.

        Returns:
            True if command should finish
        """
        # Invalid goal
        if self._goal_pose is None:
            return True

        # Success: reached goal with stable tolerance
        if self._current_state == self.STATE_FINISHED:
            return True

        # Timeout: overall command timeout
        if Timer.getFPGATimestamp() - self._start_time > self.MAX_TIMEOUT:
            SmartDashboard.putString("VisionAlign/EndReason", "Overall timeout")
            return True

        # Timeout: vision loss during PID phase
        if self._current_state == self.STATE_DIRECT_PID:
            time_without_vision = Timer.getFPGATimestamp() - self._last_vision_update_time
            if time_without_vision > self.MAX_TIME_WITHOUT_VISION:
                SmartDashboard.putString("VisionAlign/EndReason", "Vision timeout in PID phase")
                return True

        return False

    def end(self, interrupted: bool):
        """
        Called when the command ends.

        Args:
            interrupted: Whether the command was interrupted
        """
        # Cancel pathfinding command if it's running
        if self._pathfinding_command is not None:
            self._pathfinding_command.cancel()
            self._pathfinding_command = None

        # Stop the drivetrain
        self._drivetrain.set_control(
            self._drivetrain._apply_robot_speeds.with_speeds(ChassisSpeeds(0, 0, 0))
        )

        if interrupted:
            SmartDashboard.putString("VisionAlign/EndReason", "Interrupted")
        elif self._current_state == self.STATE_FINISHED:
            SmartDashboard.putString("VisionAlign/EndReason", "Success")

        SmartDashboard.putString("VisionAlign/State", "IDLE")
