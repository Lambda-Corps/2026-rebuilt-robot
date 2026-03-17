#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import commands2
import commands2.cmd
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from pathplannerlib.auto import AutoBuilder, NamedCommands, PathPlannerAuto
from phoenix6 import swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d, Translation2d

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

    TARGET_SHOOTER_DUTY_CYCLE = 0.0
    SHOOTER_SPEED_INCREMENT = 0.025
    SHOOTER_SPEED_MIN = -0.5
    
    # Track whether we're in field-centric mode
    IS_FIELD_CENTRIC = True

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
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )

        # Tower-facing request — auto-rotates toward the alliance tower while
        # still allowing full translation from the left stick.
        self._face_tower = (
            swerve.requests.FieldCentricFacingAngle()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self._face_tower.heading_controller.setPID(7.0, 0.0, 0.0)
        self._face_tower.heading_controller.enableContinuousInput(-math.pi, math.pi)

        # Alliance tower field positions (inches → meters)
        _tower_x_blue = 182 * 0.0254    # 4.623 m from blue wall
        _tower_x_red = 469 * 0.0254    # 4.623 m from blue wall
        _tower_y      = 158.84 * 0.0254 # 4.034 m from either side (mid-field)
        try:
            from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
            _field_length = AprilTagFieldLayout.loadField(
                AprilTagField.k2026RebuiltWelded
            ).getFieldLength()
        except Exception:
            _field_length = 17.548  # fallback: 2025 Reefscape field length (m)
        self._BLUE_TOWER = Translation2d(_tower_x_blue, _tower_y)
        self._RED_TOWER  = Translation2d(_tower_x_red, _tower_y)

        self._logger = Telemetry(self._max_speed)
        self._driver_controller = CommandXboxController(0)
        self._partner_controller = CommandXboxController(1)
        self.drivetrain = TunerConstants.create_drivetrain()
        self._ledsubsystem = LEDSubsystem()
        self._intake =  Intake()
        self._shooter = Shooter()
        self._ledsubsystem.setDefaultCommand(LEDCommand( self._ledsubsystem, self._shooter, self._intake))
        # Path follower
        self.configure_path_planner()

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

        move_speed_reduction = 0.6    #### Added to reduce speed while learning about swerve
        rotate_speed_reduction = 1.0  ###  NOTE THAT updating _max_speed did not seem to affect speed
        dead_zone = 0.055
        exp_scaling = 1.3

        # Default drive mode
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    (
                        self._drive_field_centric
                        if self.IS_FIELD_CENTRIC
                        else self._drive_robot_centric
                    )
                    .with_velocity_x(
                        # -self._driver_controller.getLeftY() * self._max_speed  * move_speed_reduction
                        -self.apply_deadzone_and_curve(
                            self._driver_controller.getLeftY(), dead_zone, exp_scaling
                        )
                        * self._max_speed
                        * move_speed_reduction
                        #### DF:  Updated:  Negated
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        # -self._driver_controller.getLeftX() * self._max_speed * move_speed_reduction
                        -self.apply_deadzone_and_curve(
                            self._driver_controller.getLeftX(), dead_zone, exp_scaling
                        )
                        * self._max_speed
                        * move_speed_reduction
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        # -self._driver_controller.getRightX() * self._max_angular_rate    #### DF:  Original
                        -self.apply_deadzone_and_curve(
                            self._driver_controller.getRawAxis(2) if wpilib.RobotBase.isSimulation() else self._driver_controller.getRightX(), dead_zone, exp_scaling
                        )
                        * self._max_angular_rate
                        * rotate_speed_reduction
                        #### DF:  Updated:  Negated
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        # Auto-aim at tower
        # Right bumper: hold to auto-rotate toward the alliance tower.
        # Translation (left stick) still works normally while held.
        self._driver_controller.rightBumper().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._face_tower
                    .with_velocity_x(
                        -self.apply_deadzone_and_curve(
                            self._driver_controller.getLeftY(), dead_zone, exp_scaling
                        ) * self._max_speed * move_speed_reduction
                    )
                    .with_velocity_y(
                        -self.apply_deadzone_and_curve(
                            self._driver_controller.getLeftX(), dead_zone, exp_scaling
                        ) * self._max_speed * move_speed_reduction
                    )
                    .with_target_direction(self._get_tower_direction())
            )
        )


        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )
        #Consider chaining flywheel after indexer
        self._driver_controller.a().onTrue(ControlFlywheel(self._shooter, -0.6))
        self._driver_controller.b().onTrue(ControlFlywheel(self._shooter, 0))
        self._driver_controller.button(1).onTrue(ControlFlywheel(self._shooter, -0.6))
        self._driver_controller.button(2).onTrue(ControlFlywheel(self._shooter, 0))
        self._driver_controller.leftTrigger().whileTrue(
            commands2.cmd.runOnce(lambda: setattr(self, 'TARGET_SHOOTER_DUTY_CYCLE', 0.65))
            .andThen(ControlIntake(self._intake, self.TARGET_SHOOTER_DUTY_CYCLE, False)))
        self._driver_controller.rightTrigger().whileTrue(
            commands2.cmd.runOnce(lambda: setattr(self, 'TARGET_SHOOTER_DUTY_CYCLE', 0.0))
            .andThen(ControlIntake(self._intake, self.TARGET_SHOOTER_DUTY_CYCLE, False)))
        self._driver_controller.start().toggleOnTrue(LEDrainbow(self._ledsubsystem))
        #self._driver_controller.leftTrigger().whileFalse(ControlIntake(self._intake, False, False))

        self._driver_controller.povUp().onTrue(
            commands2.cmd.runOnce(lambda: self._shooter.change_speed_variable_function(-self.SHOOTER_SPEED_INCREMENT))
        )
        self._driver_controller.povDown().onTrue(
            commands2.cmd.runOnce(lambda: self._shooter.change_speed_variable_function(self.SHOOTER_SPEED_INCREMENT))
        )

        self._partner_controller.povUp().onTrue(
            commands2.cmd.runOnce(lambda: self._shooter.change_speed_variable_function(-self.SHOOTER_SPEED_INCREMENT))
        )
        self._partner_controller.povDown().onTrue(
            commands2.cmd.runOnce(lambda: self._shooter.change_speed_variable_function(self.SHOOTER_SPEED_INCREMENT))
        )

        self._partner_controller.leftBumper().whileTrue(ControlIndexer(self._shooter, 0.6))
        self._partner_controller.rightBumper().whileTrue(ControlIndexer(self._shooter, 0))
        self._partner_controller.a().onTrue(ControlFlywheel(self._shooter, -0.6))
        self._partner_controller.b().onTrue(ControlFlywheel(self._shooter, 0))
        self._partner_controller.x().onTrue(ControlIntake(self._intake, .65, False))
        self._partner_controller.y().onTrue(ControlIntake(self._intake, .65, True))
        #self._partner_controller.y().onFalse(ControlIntake(self._intake, False, True))

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
        self.IS_FIELD_CENTRIC = not self.IS_FIELD_CENTRIC
        mode_name = "FieldCentric" if self.IS_FIELD_CENTRIC else "RobotCentric"
        log_smartdashboard_string("Drive Mode", mode_name, min_verbosity=1)
        log_debug(f"Drive mode switched to: {mode_name}", min_verbosity=1)

    def _get_tower_direction(self) -> Rotation2d:
        """Return the field-relative angle from the robot's predicted pose to its alliance tower.

        Uses velocity-based lookahead to compensate for heading PID lag
        during lateral motion.  Tunable range: 0.04–0.12 s.
        """
        LOOKAHEAD_S = 0.14  # ms prediction window

        state = self.drivetrain.get_state()
        pose = state.pose
        speeds = state.speeds  # ChassisSpeeds (robot-relative vx, vy, omega)

        tower = (
            self._RED_TOWER
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed
            else self._BLUE_TOWER
        )

        # ChassisSpeeds are robot-relative; rotate into field frame
        heading = pose.rotation().radians()
        field_vx = speeds.vx * math.cos(heading) - speeds.vy * math.sin(heading)
        field_vy = speeds.vx * math.sin(heading) + speeds.vy * math.cos(heading)

        # Predict where the robot will be in LOOKAHEAD_S seconds
        pred_x = pose.x + field_vx * LOOKAHEAD_S
        pred_y = pose.y + field_vy * LOOKAHEAD_S

        angle = math.atan2(tower.y - pred_y, tower.x - pred_x)

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            angle += math.pi

        return Rotation2d(angle)

    def apply_deadzone_and_curve(
        self, axis_value: float, deadzone: float = 0.1, exponent: float = 2.0
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
        NamedCommands.registerCommand("startflywheelStart", ControlFlywheel(self._shooter, -.6))
        NamedCommands.registerCommand("startflywheelStop", ControlFlywheel(self._shooter, -.0))
        NamedCommands.registerCommand("runindexer", ControlIndexer(self._shooter, 0.6))
        NamedCommands.registerCommand("stopIndexer", ControlIndexer(self._shooter, 0))
        NamedCommands.registerCommand("runIntake", ControlIntake(self._intake, 0.65, False))
        NamedCommands.registerCommand("stopIntake", ControlIntake(self._intake, 0, False))
        
        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Right auto")
        self._auto_chooser.addOption("Left auto", PathPlannerAuto("Left Auto"))
        self._auto_chooser.addOption("Left auto", PathPlannerAuto("Mid Auto"))
        # self._auto_chooser.addOption("Climber Test 1", PathPlannerAuto("Climber Test 1"))
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

    def shooter_speed_change(self, speed_change: float):
        self.TARGET_SHOOTER_DUTY_CYCLE = self.TARGET_SHOOTER_DUTY_CYCLE - speed_change
        
        # Clamp speeds
        if self.TARGET_SHOOTER_DUTY_CYCLE > self.SHOOTER_SPEED_MIN:
            self.TARGET_SHOOTER_DUTY_CYCLE = self.SHOOTER_SPEED_MIN
        elif self.TARGET_SHOOTER_DUTY_CYCLE < -1:
            self.TARGET_SHOOTER_DUTY_CYCLE = -1
        
        print(f"FLYWHEEL_UP +: {self.TARGET_SHOOTER_DUTY_CYCLE} ({speed_change})")
        self._shooter.flywheel_spin(self.TARGET_SHOOTER_DUTY_CYCLE)
