#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.cmd
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from pathplannerlib.auto import AutoBuilder, NamedCommands, PathPlannerAuto
from phoenix6 import swerve
from wpilib import DriverStation, RobotBase, SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.units import rotationsToRadians

from constants import (
    DEAD_ZONE,
    DEFAULT_DEADZONE,
    DEFAULT_EXPONENT,
    EXP_SCALING,
    FLYWHEEL_SPEED_INCREMENT,
    INDEXER_SPEED,
    INTAKE_SPEED,
    MAX_ANGULAR_VELOCITY_ROTATIONS,
    MOVE_SPEED_REDUCTION,
    ROTATE_DEAD_ZONE,
    ROTATE_SPEED_REDUCTION,
    ROTATIONAL_DEADBAND_FACTOR,
    SHOOTER_SPEED,
    VISION_DEAD_ZONE,
    VISION_EXP_SCALING,
    VISION_MOVE_SPEED_REDUCTION,
    VISION_ROTATE_DEAD_ZONE,
    VISION_ROTATE_SPEED_REDUCTION,
)
from subsystems.ledsubsystem import LEDSubsystem
from commands.ledcommand import LEDCommand
from commands.LEDrainbow import LEDrainbow

from subsystems.VisionSubsystem import VisionSubsystem
from commands.VisionTrackTargetPair import VisionTrackTargetPair

from commands.intakeCommand import ControlIntake

from commands.indexerCommand import ControlIndexer

from commands.ShooterCommand import ShooterCommand  # newer, used by auto-aim
from commands.changeSpeedFlywheel import ChangeFlywheelSpeed
from commands.flywheelCommand import ControlFlywheel   # Used by autonomous pathfinder.
# TODO:  Need to update pathfinder to enable automatic vision tracking
from commands.flywheelStopCommand import StopFlywheel
from commands.resetPDHStickyFaults import ResetPDHStickyFaults

from subsystems.shooter_subsystem import ShooterSubsystem

# DF: Added to quiet Console log
import wpilib
from wpilib import LiveWindow


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # ================================================================
        # DF: Added to quiet Console log

        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        # The function FRC LiveWindow.disableAllTelemetry() is a static method
        # in the FRC (FIRST Robotics Competition) WPILib library that disables t
        # he sending of data for all sensors and actuators to the SmartDashboard
        # or Shuffleboard LiveWindow display.

        LiveWindow.disableAllTelemetry()

        # https://robotpy.readthedocs.io/projects/robotpy/en/latest/wpilib/LiveWindow.html

        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(MAX_ANGULAR_VELOCITY_ROTATIONS)

        # Track whether we're in field-centric mode
        self._is_field_centric = False
        self._button_bindings_configured = False

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive_robot_centric = (
            swerve.requests.RobotCentric()
            .with_rotational_deadband(
                self._max_angular_rate * ROTATIONAL_DEADBAND_FACTOR
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._drive_field_centric = (
            swerve.requests.FieldCentric()
            .with_rotational_deadband(
                self._max_angular_rate * ROTATIONAL_DEADBAND_FACTOR
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = swerve.requests.RobotCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )

        self._logger = Telemetry(self._max_speed)

        self._driver_controller = CommandXboxController(0)
        self._partner_controller = CommandXboxController(1)

        # Create GenericHIDSim only in simulation mode
        self._driver_controller_sim = None
        if RobotBase.isSimulation():
            from wpilib.simulation import GenericHIDSim

            self._driver_controller_sim = GenericHIDSim(0)

        self.drivetrain = TunerConstants.create_drivetrain()

        self._ledsubsystem = LEDSubsystem()
        self._shooter_subsystem = ShooterSubsystem()
        self._ledsubsystem.setDefaultCommand(LEDCommand( self._ledsubsystem, self._shooter_subsystem, self._shooter_subsystem))
        self._vision_subsystem = VisionSubsystem(self.drivetrain)

        # Path follower
        # self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        # SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Set up default commands
        self.setDefaultCommands()
        self.configure_path_planner()

    def setDefaultCommands(self) -> None:
        """Set up default commands for subsystems. Called from __init__."""
        # Always set up default command first (so robot appears in Field2d)

        joystick = self._driver_controller

        def get_drive_request():
            # Read right stick X: axis 2 in simulation, axis 4 on real hardware
            if RobotBase.isSimulation():
                rot_axis = joystick._hid.getRawAxis(2)
            else:
                rot_axis = joystick._hid.getRawAxis(4)

            rot_rate = (
                -self.apply_deadzone_and_curve(rot_axis, ROTATE_DEAD_ZONE, EXP_SCALING)
                * self._max_angular_rate
                * ROTATE_SPEED_REDUCTION
            )

            return (
                (
                    self._drive_field_centric
                    if self._is_field_centric
                    else self._drive_robot_centric
                )
                .with_velocity_x(
                    -self.apply_deadzone_and_curve(
                        joystick.getLeftY(), DEAD_ZONE, EXP_SCALING
                    )
                    * self._max_speed
                    * MOVE_SPEED_REDUCTION
                )
                .with_velocity_y(
                    -self.apply_deadzone_and_curve(
                        joystick.getLeftX(), DEAD_ZONE, EXP_SCALING
                    )
                    * self._max_speed
                    * MOVE_SPEED_REDUCTION
                )
                .with_rotational_rate(rot_rate)
            )

        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: get_drive_request())
        )

        # self._ledsubsystem.setDefaultCommand(LEDCommand(self._ledsubsystem, 0.5))    ##  TODO: Revise LED operation

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def configureButtonBindings(self) -> None:
        """Set up button bindings. Call this from teleopInit when joystick is ready."""
        if self._button_bindings_configured:
            return
        self._button_bindings_configured = True
        print("[RobotContainer] Setting up button bindings...")

        # Speed settings for translation
        move_speed_reduction = VISION_MOVE_SPEED_REDUCTION
        dead_zone = VISION_DEAD_ZONE
        exp_scaling = VISION_EXP_SCALING

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        def get_translation():
            # Returns (vx, vy) matching ChassisSpeeds convention:
            #   vx = forward/back (from left stick Y)
            #   vy = strafe left/right (from left stick X)
            return (
                -self.apply_deadzone_and_curve(
                    self._driver_controller.getLeftY(), dead_zone, exp_scaling
                )
                * self._max_speed
                * move_speed_reduction,
                -self.apply_deadzone_and_curve(
                    self._driver_controller.getLeftX(), dead_zone, exp_scaling
                )
                * self._max_speed
                * move_speed_reduction,
            )

        def get_drive_request():
            # Returns the appropriate SwerveRequest based on current drive mode
            # This ensures vision tracking uses same control mode as normal driving
            if self._is_field_centric:
                return self._drive_field_centric
            return self._drive_robot_centric

        def get_rotation():
            # Returns rotation rate (rad/s) from right stick for driver control
            if RobotBase.isSimulation():
                rot_axis = self._driver_controller._hid.getRawAxis(2)
            else:
                rot_axis = self._driver_controller._hid.getRawAxis(4)
            return (
                -self.apply_deadzone_and_curve(rot_axis, VISION_ROTATE_DEAD_ZONE, exp_scaling)
                * self._max_angular_rate
                * VISION_ROTATE_SPEED_REDUCTION
            )

        ## Driver controller bindings

        # Toggle between RobotCentric and FieldCentric on left bumper press
        self._driver_controller.leftBumper().onTrue(commands2.cmd.runOnce(lambda: self._toggle_drive_mode()))

        # Test? Remove?
        self._driver_controller.start().toggleOnTrue(LEDrainbow(self._ledsubsystem))

        # Intake
        self._driver_controller.leftTrigger().whileTrue(ControlIntake(self._shooter_subsystem, INTAKE_SPEED, False))
        # self._driver_controller.rightTrigger().whileTrue(ControlIntake(self._shooter_subsystem, 0, False))
        
        # Indexer
        self._driver_controller.x().onTrue(ControlIndexer(self._shooter_subsystem, INDEXER_SPEED))
        self._driver_controller.y().onTrue(ControlIndexer(self._shooter_subsystem, -INDEXER_SPEED))

        # Manual shooter on/off
        self._driver_controller.a().onTrue(ControlFlywheel(self._shooter_subsystem, -SHOOTER_SPEED))
        self._driver_controller.b().onTrue(StopFlywheel(self._shooter_subsystem))

        # # Auto Shooter Right Trigger
        # self._driver_controller.rightTrigger().whileTrue(
        #     commands2.ParallelCommandGroup(
        #         VisionTrackTargetPair(
        #             self.drivetrain,
        #             self._vision_subsystem,
        #             get_translation,
        #             get_drive_request,
        #             get_rotation,
        #         ),
        #         ControlFlywheel(self._shooter_subsystem, -SHOOTER_SPEED),
        #         ControlIndexer(self._shooter_subsystem, INDEXER_SPEED, True)
        #     )
        # )

        # Activates auto-aim, shooter, and indexer together while right trigger is held.
        # Passes callbacks to get translation input and drive request based on current mode (field-centric or robot-centric)
        # In simulation, the right trigger maps to axis 5 instead of the default axis
        if RobotBase.isSimulation():
            right_trigger = Trigger(lambda: self._driver_controller._hid.getRawAxis(5) > 0.5)
            get_right_trigger_axis = lambda: self._driver_controller._hid.getRawAxis(5)
        else:
            right_trigger = self._driver_controller.rightTrigger()
            get_right_trigger_axis = lambda: self._driver_controller.getRightTriggerAxis()

        right_trigger.whileTrue(
            commands2.ParallelCommandGroup(
                VisionTrackTargetPair(
                    self.drivetrain,
                    self._vision_subsystem,
                    get_translation,
                    get_drive_request,
                    get_rotation,
                ),
                ShooterCommand(
                    self._shooter_subsystem,
                    self._vision_subsystem,
                    get_right_trigger_axis,
                )
            )
        )

        # Toggle Auto-aim and shooter speed
        self._driver_controller.rightBumper().toggleOnTrue(
            commands2.ParallelCommandGroup(
                VisionTrackTargetPair(
                    self.drivetrain,
                    self._vision_subsystem,
                    get_translation,
                    get_drive_request,
                    get_rotation,
                )
            ))
        # self._driver_controller.rightBumper().whileTrue(LEDCommand(self._ledsubsystem, 120))
        # self._driver_controller.rightBumper().whileFalse(LEDCommand(self._ledsubsystem, 0))



        ## Partner contoller bindings

        # Indexer
        self._partner_controller.leftBumper().whileTrue(ControlIndexer(self._shooter_subsystem, INDEXER_SPEED))
        self._partner_controller.rightBumper().whileTrue(ControlIndexer(self._shooter_subsystem, 0))

        # Shooter on/off
        self._partner_controller.a().onTrue(ShooterSubsystem.set_shooter_speed(self._shooter_subsystem, -SHOOTER_SPEED))
        self._partner_controller.b().onTrue(ShooterSubsystem.set_shooter_speed(self._shooter_subsystem, 0))
        
        # Indexer on/off
        self._partner_controller.x().onTrue(ControlIntake(self._shooter_subsystem, INTAKE_SPEED, False))
        self._partner_controller.y().onTrue(ControlIntake(self._shooter_subsystem, INTAKE_SPEED, True))

        # During auto-aim, speed will be controled based on distance to target
        self._partner_controller.povUp().onTrue(ChangeFlywheelSpeed(self._shooter_subsystem, FLYWHEEL_SPEED_INCREMENT))
        self._partner_controller.povDown().onTrue(ChangeFlywheelSpeed(self._shooter_subsystem, -FLYWHEEL_SPEED_INCREMENT))

        self._partner_controller.leftTrigger().onTrue(ControlIntake(self._shooter_subsystem, .65, True))
        self._partner_controller.start().toggleOnTrue(ResetPDHStickyFaults(self._shooter_subsystem))

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def _initialize_robot_pose(self):
        """Initialize robot pose based on starting position and alliance."""
        alliance = DriverStation.getAlliance()
        is_red = alliance == DriverStation.Alliance.kRed

        # TODO: Make this configurable via SmartDashboard or auto-chooser
        starting_position_id = 1

        # Define starting poses (adjust for 2026 field)
        starting_poses = {
            1: Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            2: Pose2d(1.0, 4.0, Rotation2d.fromDegrees(0)),
            3: Pose2d(1.0, 7.0, Rotation2d.fromDegrees(0)),
            4: Pose2d(1.0, 10.0, Rotation2d.fromDegrees(0)),
        }

        initial_pose = starting_poses.get(
            starting_position_id, Pose2d(0, 0, Rotation2d(0))
        )

        # Mirror for red alliance if needed
        if is_red:
            field_width = 16.54
            initial_pose = Pose2d(
                field_width - initial_pose.X(),          #  Not absolutely sure this solves the crashing problem. Error moved to line 386
                # field_width - initial_pose.x(),
                initial_pose.Y(),
                # initial_pose.y(),
                Rotation2d.fromDegrees(180) - initial_pose.rotation(),
            )

        # Reset pose estimator
        # self.drivetrain.seedFieldRelative(initial_pose)  DF:  TODO: resolve "seedFieldRelative"
        self.drivetrain.reset_pose(initial_pose)
        # self.drivetrain.seedFieldRelative(initial_pose)
        alliance_name = "Red" if is_red else "Blue"
        SmartDashboard.putString(
            "Robot/InitialPose", f"Pos {starting_position_id}, {alliance_name}"
        )

    def _toggle_drive_mode(self) -> None:
        """Toggle between field-centric and robot-centric drive modes."""
        self._is_field_centric = not self._is_field_centric
        mode_str = "Field Centric" if self._is_field_centric else "Robot Centric"
        SmartDashboard.putString("Drive/Mode", mode_str)

    @staticmethod
    def apply_deadzone_and_curve(
        axis_value: float,
        deadzone: float = DEFAULT_DEADZONE,
        exponent: float = DEFAULT_EXPONENT,
    ) -> float:
        if abs(axis_value) < deadzone:
            return 0.0
        # Normalize to 0-1 range after deadzone
        normalized = (abs(axis_value) - deadzone) / (1.0 - deadzone)
        # Apply curve (e.g., square for smoother ramp)
        curved = normalized**exponent
        # Reapply sign
        final = curved * (1 if axis_value > 0 else -1)
        return final

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()


    def configure_path_planner(self):

        # Named commands must be created before Autos can be defined
        NamedCommands.registerCommand("startflywheelStart", ControlFlywheel(self._shooter_subsystem, -0.6))
        NamedCommands.registerCommand("runindexer", ControlIndexer(self._shooter_subsystem, 0.6))
        NamedCommands.registerCommand("startflywheelStop", ControlIndexer(self._shooter_subsystem, 0.0))
        NamedCommands.registerCommand("run_Intake",ControlIntake(self._shooter_subsystem, 0.65, False))
        NamedCommands.registerCommand("ResetPDHStickyFaults",ResetPDHStickyFaults(self._shooter_subsystem))
        
        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("midfieldshoot")  # Parameter is default Path
        self._auto_chooser.addOption("midfieldshoot", PathPlannerAuto("midfieldshoot"))
        self._auto_chooser.addOption("outpost auto blue", PathPlannerAuto("outpost auto blue"))
        self._auto_chooser.addOption("get+shoot+get fuel+climb", PathPlannerAuto("get+shoot+get fuel+climb"))
        SmartDashboard.putData("Auto Mode", self._auto_chooser)
