#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.cmd
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from commands import changeSpeedFlywheel
from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from pathplannerlib.auto import AutoBuilder
from phoenix6 import swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d

from utils.logger import log_debug, log_smartdashboard_string

from utils.logger import log_debug, log_smartdashboard_string
from wpimath.units import rotationsToRadians

from subsystems.ledsubsystem import LEDSubsystem
from commands.ledcommand import LEDCommand

from commands.indexerCommand import ControlIndexer
from commands.intakeCommand import ControlIntake
from commands.flywheelCommand import ControlFlywheel

from subsystems.shooter import Shooter

from subsystems.intake import Intake

from commands.LEDrainbow import LEDrainbow

from commands.changeSpeedFlywheel import ChangeFlywheelSpeed
#================================================================
# DF: Added to quiet Console log
import wpilib
from wpilib import LiveWindow
#================================================================

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

#================================================================
# DF: Added to quiet Console log

        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        # The function FRC LiveWindow.disableAllTelemetry() is a static method 
        # in the FRC (FIRST Robotics Competition) WPILib library that disables t
        # he sending of data for all sensors and actuators to the SmartDashboard 
        # or Shuffleboard LiveWindow display.  

        LiveWindow.disableAllTelemetry()
        
        # https://robotpy.readthedocs.io/projects/robotpy/en/latest/wpilib/LiveWindow.html        

        #    We are getting a number of Watch Dog errors due to excessive time being taken up by RobotPeriodic().
        #    Near line 320, is a command "register_telemetry" which appears to cause the motors to create and log
        #    telemetry  data.  By removing  this line, the errors have disappeared.   This is needed for simulation.
        #    NOTE: It appears some CTRE settings require power cycling to update the configuration.
        #
#============================================================================




        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.85
        )  # 3/4 of a rotation per second max angular velocity

        # Track whether we're in field-centric mode
        self._is_field_centric = False

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive_robot_centric = (
            swerve.requests.RobotCentric()
            .with_rotational_deadband(
                self._max_angular_rate * 0.075
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._drive_field_centric = (
            swerve.requests.FieldCentric()
            .with_rotational_deadband(
                self._max_angular_rate * 0.075
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = swerve.requests.RobotCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        self._forward_straight = swerve.requests.RobotCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )

        self._logger = Telemetry(self._max_speed)
        self._driver_controller = CommandXboxController(0)
        self._partner_controller = CommandXboxController(1)
        self.drivetrain = TunerConstants.create_drivetrain()
        self._ledsubsystem = LEDSubsystem()
        self._intake =  Intake()
        self._shooter = Shooter()
        # self._intake.setDefaultCommand(ControlIntake(self._intake, False, False))
        # self._shooter.setDefaultCommand(ControlFlywheel(self._shooter, 0))
        # self._shooter.setDefaultCommand(ControlIndexer(self._shooter, 0))
        self._ledsubsystem.setDefaultCommand(LEDCommand( self._ledsubsystem, self._shooter, self._intake))
        

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.

        move_speed_reduction = 0.48    #### Added to reduce speed while learning about swerve
        rotate_speed_reduction = 1.0  ###  NOTE THAT updating _max_speed did not seem to affect speed
        move_speed_reduction = (
            0.8  #### Added to reduce speed while learning about swerve
        )
        rotate_speed_reduction = (
            1.0  ###  NOTE THAT updating _max_speed did not seem to affect speed
        )
        dead_zone = 0.055
        exp_scaling = 1.3
        exp_scaling = 1.4

        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    (
                        self._drive_field_centric
                        if self._is_field_centric
                        else self._drive_robot_centric
                    )
                    .with_velocity_x(
                        # -self._joystick.getLeftY() * self._max_speed  * move_speed_reduction
                        -self.apply_deadzone_and_curve(
                            self._driver_controller.getLeftY(), dead_zone, exp_scaling
                        )
                        * self._max_speed
                        * move_speed_reduction
                        #### DF:  Updated:  Negated
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        # -self._joystick.getLeftX() * self._max_speed * move_speed_reduction
                        -self.apply_deadzone_and_curve(
                            self._driver_controller.getLeftX(), dead_zone, exp_scaling
                        )
                        * self._max_speed
                        * move_speed_reduction
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        # -self._joystick.getRightX() * self._max_angular_rate    #### DF:  Original
                        -self._driver_controller.getRightX()
                        * self._max_angular_rate
                        * rotate_speed_reduction
                        #### DF:  Updated:  Negated
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        self._ledsubsystem.setDefaultCommand(LEDCommand(self._ledsubsystem, 0.5))

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )
 
        self._driver_controller.a().onTrue(ControlFlywheel(self._shooter, -0.6))
        self._driver_controller.b().onTrue(ControlFlywheel(self._shooter, 0))
        self._driver_controller.leftTrigger().whileTrue(ControlIntake(self._intake, True, False))
        self._driver_controller.start().toggleOnTrue(LEDrainbow(self._ledsubsystem))
        #self._driver_controller.leftTrigger().whileFalse(ControlIntake(self._intake, False, False))

        self._partner_controller.leftBumper().whileTrue(ControlIndexer(self._shooter, 0.6))
        self._partner_controller.rightBumper().whileTrue(ControlIndexer(self._shooter, 0))
        self._partner_controller.a().onTrue(ControlFlywheel(self._shooter, -0.6))
        self._partner_controller.b().onTrue(ControlFlywheel(self._shooter, 0))
        self._partner_controller.x().onTrue(ControlIntake(self._intake, True, False))
        #self._partner_controller.x().onFalse(ControlIntake(self._intake, False, False))
        self._partner_controller.y().onTrue(ControlIntake(self._intake, False, True))
        #self._partner_controller.y().onFalse(ControlIntake(self._intake, False, True))

        self._driver_controller.povUp().onTrue(ChangeFlywheelSpeed(self._shooter, 0.05))        

        self._driver_controller.povDown().onTrue(ChangeFlywheelSpeed(self._shooter, -0.05))

        # self._driver_controller.pov(0).whileTrue(
        #     self.drivetrain.apply_request(
        #         lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0)
        #     )
        # )
        # self._driver_controller.pov(180).whileTrue(
        #     self.drivetrain.apply_request(
        #         lambda: self._forward_straight.with_velocity_x(-0.5).with_velocity_y(0)
        #     )
        # )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._driver_controller.back() & self._driver_controller.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._driver_controller.back() & self._driver_controller.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._driver_controller.start() & self._driver_controller.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._driver_controller.start() & self._driver_controller.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # Toggle between RobotCentric and FieldCentric on left bumper press
        self._driver_controller.leftBumper().onTrue(
            commands2.cmd.runOnce(lambda: self._toggle_drive_mode())
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def _toggle_drive_mode(self) -> None:
        """Toggle between RobotCentric and FieldCentric drive modes."""
        self._is_field_centric = not self._is_field_centric
        mode_name = "FieldCentric" if self._is_field_centric else "RobotCentric"
        log_smartdashboard_string("Drive Mode", mode_name, min_verbosity=1)
        log_debug(f"Drive mode switched to: {mode_name}", min_verbosity=1)
        log_smartdashboard_string("Drive Mode", mode_name, min_verbosity=1)
        log_debug(f"Drive mode switched to: {mode_name}", min_verbosity=1)

    def apply_deadzone_and_curve(
        self, axis_value: float, deadzone: float = 0.1, exponent: float = 2.0
    ) -> float:
        if abs(axis_value) < deadzone:
            return 0.0
        # Normalize to 0-1 range after deadzone
        normalized = (abs(axis_value) - deadzone) / (1.0 - deadzone)
        # Apply curve (e.g., square for smoother ramp)
        curved = normalized**exponent
        curved = normalized**exponent
        # Reapply sign
        final = curved * (1 if axis_value > 0 else -1)
        return final

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()
