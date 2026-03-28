#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from constants import (
    SHOOTER_DEFAULT_RPS,
    SHOOTER_MIN_RPS,
    SHOOTER_MAX_RPS,
    SHOOTER_SPEED_UP,
    SHOOTER_SPEED_RIGHT,
    SHOOTER_SPEED_DOWN,
    SHOOTER_SPEED_LEFT,
    SHOOTER_SPEED_INCREMENT,
    SHOOTER_X_OFFSET_INCHES,
    INDEXER_SPEED_DEFAULT,
    INTAKE_SPEED_DEFAULT,
    MOVE_SPEED_COEFF,
    ROTATE_SPEED_COEFF,
    JOYSTICK_DEAD_ZONE,
    JOYSTICK_EXP_SCALING,
    SHOOTER_QUADRCOEF_A,
    SHOOTER_QUADRCOEF_B,
    SHOOTER_QUADRCOEF_C
)

import commands2
import commands2.cmd
import math
import wpilib

from commands.flywheelCommand import ControlFlywheel
from commands.indexerCommand import ControlIndexer
from commands.intakeCommand import ControlIntake
from commands.ledcommand import LEDCommand
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from pathplannerlib.auto import AutoBuilder, NamedCommands
from phoenix6 import swerve
from telemetry import Telemetry
from wpilib import DriverStation, LiveWindow, SmartDashboard
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.units import rotationsToRadians

from subsystems.intake import Intake
from subsystems.ledsubsystem import LEDSubsystem
from subsystems.shooter import Shooter
from subsystems.VisionSubsystem import VisionSubsystem
from utils.logger import log_debug, log_smartdashboard_string

# ================================================================
# DF: Added to quiet Console log


# ================================================================


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    TARGET_SHOOTER_SPEED = 0.0

    # Track whether we're in field-centric mode
    IS_FIELD_CENTRIC = True

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

        #    We are getting a number of Watch Dog errors due to excessive time being taken up by RobotPeriodic().
        #    Near line 320, is a command "register_telemetry" which appears to cause the motors to create and log
        #    telemetry  data.  By removing  this line, the errors have disappeared.   This is needed for simulation.
        #    NOTE: It appears some CTRE settings require power cycling to update the configuration.
        #
        # ============================================================================

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
            swerve.requests.FieldCentricFacingAngle().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        self._face_tower.heading_controller.setPID(7.0, 0.0, 0.0)
        self._face_tower.heading_controller.enableContinuousInput(-math.pi, math.pi)

        # Alliance tower field positions (inches → meters)
        _tower_x_blue = 182 * 0.0254  # 4.623 m from blue wall
        _tower_x_red = 469 * 0.0254  # 4.623 m from blue wall
        _tower_y = 158.84 * 0.0254  # 4.034 m from either side (mid-field)
        try:
            from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

            _field_length = AprilTagFieldLayout.loadField(
                AprilTagField.k2026RebuiltWelded
            ).getFieldLength()
        except Exception:
            _field_length = 17.548  # fallback: 2025 Reefscape field length (m)
        self._BLUE_TOWER = Translation2d(_tower_x_blue, _tower_y)
        self._RED_TOWER = Translation2d(_tower_x_red, _tower_y)

        # Placeholder coordinates for the secondary targets to aim at when out of the "alliance area"
        # Update these coordinates to point to the actual desired field locations
        ALT_TARGET_X_OFFSET = 1.0
        ALT_TARGET_Y_OFFSET = 1.0
        self._BLUE_SECONDARY_TARGET_A = Translation2d(_tower_x_blue - ALT_TARGET_X_OFFSET, _tower_y + ALT_TARGET_Y_OFFSET)
        self._BLUE_SECONDARY_TARGET_B = Translation2d(_tower_x_blue - ALT_TARGET_X_OFFSET, _tower_y - ALT_TARGET_Y_OFFSET)
        self._RED_SECONDARY_TARGET_A = Translation2d(_tower_x_red + ALT_TARGET_X_OFFSET, _tower_y + ALT_TARGET_Y_OFFSET)
        self._RED_SECONDARY_TARGET_B = Translation2d(_tower_x_red + ALT_TARGET_X_OFFSET, _tower_y - ALT_TARGET_Y_OFFSET)

        self._logger = Telemetry(self._max_speed)
        self._driver_controller = CommandXboxController(0)
        self._partner_controller = CommandXboxController(1)
        self.drivetrain = TunerConstants.create_drivetrain()
        self._ledsubsystem = LEDSubsystem()
        self._intake = Intake()
        self._shooter = Shooter()
        self._vision = VisionSubsystem(self.drivetrain)
        self._target_distance = (
            0.0  # Initialized here, updated by _get_tower_direction()
        )
        self._ledsubsystem.setDefaultCommand(
            LEDCommand(self._ledsubsystem, self._shooter, self._intake)
        )
        # Path follower
        self.configure_path_planner()

        # Configure the button bindings
        self.configureButtonBindings()

    def getAutonomousCommand():
        return self.autoChooser.getSelected()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.

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
                        # -self._driver_controller.getLeftY() * self._max_speed  * MOVE_SPEED_COEFF
                        -self.apply_deadzone_and_curve(
                            self._driver_controller.getLeftY(), JOYSTICK_DEAD_ZONE, JOYSTICK_EXP_SCALING
                        )
                        * self._max_speed
                        * MOVE_SPEED_COEFF
                        #### DF:  Updated:  Negated
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        # -self._driver_controller.getLeftX() * self._max_speed * MOVE_SPEED_COEFF
                        -self.apply_deadzone_and_curve(
                            self._driver_controller.getLeftX(), JOYSTICK_DEAD_ZONE, JOYSTICK_EXP_SCALING
                        )
                        * self._max_speed
                        * MOVE_SPEED_COEFF
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self.apply_deadzone_and_curve(
                            self._driver_controller.getRightX() 
                                if not wpilib.RobotBase.isSimulation()
                                else self._driver_controller.getRawAxis(2),
                            JOYSTICK_DEAD_ZONE,
                            JOYSTICK_EXP_SCALING,
                        )
                        * self._max_angular_rate
                        * ROTATE_SPEED_COEFF
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True))

        # Driver controls
        self._driver_controller.leftBumper().onTrue(
            commands2.cmd.runOnce(lambda: self._toggle_drive_mode()))
        
        # These methods are passed to the auto-aim and distance shooter command
        teleop_vel_x = lambda: -self.apply_deadzone_and_curve(
            self._driver_controller.getLeftY(), JOYSTICK_DEAD_ZONE, JOYSTICK_EXP_SCALING
        ) * self._max_speed * MOVE_SPEED_COEFF
        teleop_vel_y = lambda: -self.apply_deadzone_and_curve(
            self._driver_controller.getLeftX(), JOYSTICK_DEAD_ZONE, JOYSTICK_EXP_SCALING
        ) * self._max_speed * MOVE_SPEED_COEFF

        # Auto-aim at tower
        # Right trigger: hold to auto-rotate toward the alliance tower.
        # Translation (left stick) still works normally while held.
        (
            self._partner_controller.rightTrigger(0.05) # Hardware
            | (Trigger(wpilib.RobotBase.isSimulation) & Trigger(lambda: self._driver_controller.getRawAxis(5) > JOYSTICK_DEAD_ZONE)) # Simulation using Xbox controller
        ).whileTrue(self.auto_aim_and_distance_shooter(teleop_vel_x, teleop_vel_y))

        # Sim "driver" controls
        self._driver_controller.a().onTrue(
            ControlFlywheel(self._shooter, -self._shooter.MOTOR_SPEED_GLOBAL))
        self._driver_controller.b().onTrue(
            ControlFlywheel(self._shooter, 0))
        self._driver_controller.button(1).onTrue(
            ControlFlywheel(self._shooter, -self._shooter.MOTOR_SPEED_GLOBAL))
        self._driver_controller.button(2).onTrue(
            ControlFlywheel(self._shooter, 0))

        # self._driver_controller.start().toggleOnTrue(LEDrainbow(self._ledsubsystem))

        # Intake controls
        self._partner_controller.x().onTrue(
            ControlIntake(self._intake, INTAKE_SPEED_DEFAULT, False))
        self._partner_controller.y().onTrue(
            ControlIntake(self._intake, INTAKE_SPEED_DEFAULT, True))
        self._partner_controller.leftTrigger().whileTrue(
            ControlIntake(self._intake, 0, False))

        # Shooter speed presets
        (self._driver_controller.pov(0) | self._partner_controller.pov(0)).onTrue(
            ControlFlywheel(self._shooter, SHOOTER_SPEED_UP))
        (self._driver_controller.pov(90) | self._partner_controller.pov(90)).onTrue(
            ControlFlywheel(self._shooter, SHOOTER_SPEED_RIGHT))
        (self._driver_controller.pov(180) | self._partner_controller.pov(180)).onTrue(
            ControlFlywheel(self._shooter, SHOOTER_SPEED_DOWN))
        (self._driver_controller.pov(270) | self._partner_controller.pov(270)).onTrue(
            ControlFlywheel(self._shooter, SHOOTER_SPEED_LEFT))

        # Shooter speed variable control
        self._partner_controller.start().onTrue(
            commands2.cmd.runOnce(
                lambda: self._shooter.change_speed_variable_function(-SHOOTER_SPEED_INCREMENT)))
        self._partner_controller.back().onTrue(
            commands2.cmd.runOnce(
                lambda: self._shooter.change_speed_variable_function(SHOOTER_SPEED_INCREMENT)))

        # Shooter start/stop
        self._partner_controller.a().onTrue(
            ControlFlywheel(self._shooter, -self._shooter.MOTOR_SPEED_GLOBAL))
        self._partner_controller.b().onTrue(
            ControlFlywheel(self._shooter, 0))

        # Indexer controls
        self._partner_controller.leftBumper().whileTrue(
            ControlIndexer(self._shooter, INDEXER_SPEED_DEFAULT))
        self._partner_controller.rightBumper().whileTrue(
            ControlIndexer(self._shooter, 0))

        self._partner_controller.rightStick().onTrue(
            commands2.cmd.runOnce(self._attempt_vision_seed))

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state))

        # This allows on-demand characterization of the drivetrain without Phoenix Tuner X, but requires you to copy values from logs to tuner_constants.py
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

    def _toggle_drive_mode(self) -> None:
        """Toggle between RobotCentric and FieldCentric drive modes."""
        self.IS_FIELD_CENTRIC = not self.IS_FIELD_CENTRIC
        mode_name = "FieldCentric" if self.IS_FIELD_CENTRIC else "RobotCentric"
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
        # Reapply sign
        final = curved * (1 if axis_value > 0 else -1)
        return final

    def _get_tower_direction(self) -> Rotation2d:
        """Return the field-relative angle from the robot's predicted pose to its target.
        If the robot is between its alliance wall and the tower, it targets the tower.
        Otherwise, it targets the alliance-specific secondary target.

        Uses velocity-based lookahead to compensate for heading PID lag
        during lateral motion.  Tunable range: 0.04–0.12 s.
        """
        LOOKAHEAD_S = 0.14  # ms prediction window

        state = self.drivetrain.get_state()
        pose = state.pose
        speeds = state.speeds  # ChassisSpeeds (robot-relative vx, vy, omega)

        is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed

        # Determine if we are in the "alliance area" (between driver station and tower)
        # Blue driver station is at x=0; Red driver station is at x ~ field_length
        if is_red:
            in_alliance_area = pose.x > self._RED_TOWER.x
        else:
            in_alliance_area = pose.x < self._BLUE_TOWER.x

        if in_alliance_area:
            # Target our aliance tower
            target = self._RED_TOWER if is_red else self._BLUE_TOWER
        else:
            # Depending on robot Y, target SECONDARY target A or B (the same side the robot is on)
            if pose.y >= self._BLUE_TOWER.y:
                target = self._RED_SECONDARY_TARGET_A if is_red else self._BLUE_SECONDARY_TARGET_A
            else:
                target = self._RED_SECONDARY_TARGET_B if is_red else self._BLUE_SECONDARY_TARGET_B

        # ChassisSpeeds are robot-relative; rotate into field frame
        heading = pose.rotation().radians()
        field_vx = speeds.vx * math.cos(heading) - speeds.vy * math.sin(heading)
        field_vy = speeds.vx * math.sin(heading) + speeds.vy * math.cos(heading)

        # Predict where the robot will be in LOOKAHEAD_S seconds
        pred_x = pose.x + field_vx * LOOKAHEAD_S
        pred_y = pose.y + field_vy * LOOKAHEAD_S

        angle = math.atan2(target.y - pred_y, target.x - pred_x)

        # Because of the rotation that is automatically applied based on aliance, we need to add 180 degrees (pi radians) to the angle when on the red alliance to ensure the robot faces the target (not backend facing).
        if is_red:
            angle += math.pi

        # Distance from the back of the robot to the target
        BACK_OFFSET_METERS = SHOOTER_X_OFFSET_INCHES * 0.0254  # inches behind center (wheel line)
        back_x = pose.x # - BACK_OFFSET_METERS * math.cos(heading)
        back_y = pose.y # - BACK_OFFSET_METERS * math.sin(heading)
        self._target_distance = math.sqrt(
            (target.x - back_x) ** 2 + (target.y - back_y) ** 2
        ) + BACK_OFFSET_METERS
        SmartDashboard.putNumber(
            "TargetDistanceMeters", round(self._target_distance, 2)
        )

        return Rotation2d(angle)

    def _flywheel_speed_from_distance(self, distance: float, voltage: float = None) -> float:
        """Return flywheel speed for a given distance using quadratic fit: a*x^2 + b*x + c,
        scaled by a voltage compensation multiplier."""
        base_speed = SHOOTER_QUADRCOEF_A * (distance ** 2) + SHOOTER_QUADRCOEF_B * distance + SHOOTER_QUADRCOEF_C

        if voltage is None:
            voltage = wpilib.RobotController.getBatteryVoltage()

        # Curve outputs target RPS cleanly now. 
        return base_speed

    def auto_aim_and_distance_shooter(self, velocity_x_supplier, velocity_y_supplier) -> commands2.Command:
        """
        Creates a command that uses _face_tower to aim at the alliance tower and
        simultaneously spins up the shooter based on distance.
        """
        return commands2.ParallelCommandGroup(
            self.drivetrain.apply_request(
                lambda: self._face_tower
                .with_velocity_x(velocity_x_supplier())
                .with_velocity_y(velocity_y_supplier())
                .with_target_direction(self._get_tower_direction())
            ),
            commands2.cmd.run(
                lambda: self._shooter.flywheel_spin(
                    self._flywheel_speed_from_distance(self._target_distance)
                ),
                self._shooter,
            )
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self.autoChooser.getSelected()

    def configure_path_planner(self):
        # Named commands must be created before Autos can be defined
        NamedCommands.registerCommand("startflywheelStart", ControlFlywheel(self._shooter, SHOOTER_DEFAULT_RPS))
        NamedCommands.registerCommand("startflywheelStop", ControlFlywheel(self._shooter, -0.0))
        NamedCommands.registerCommand("runindexer", ControlIndexer(self._shooter, INDEXER_SPEED_DEFAULT))
        NamedCommands.registerCommand("stopIndexer", ControlIndexer(self._shooter, 0))
        NamedCommands.registerCommand("runIntake", ControlIntake(self._intake, INTAKE_SPEED_DEFAULT, False))
        NamedCommands.registerCommand("stopIntake", ControlIntake(self._intake, 0, False))
        NamedCommands.registerCommand("AutoAimStationary", self.auto_aim_and_distance_shooter(lambda: 0.0, lambda: 0.0))
        NamedCommands.registerCommand("AutoAimStationary_2Sec", self.auto_aim_and_distance_shooter(lambda: 0.0, lambda: 0.0).withTimeout(2.0))
        NamedCommands.registerCommand("VisionReseed", commands2.cmd.runOnce(self._attempt_vision_seed))
        # Build an auto chooser. This will use Commands.none() as the default option.
        self.autoChooser = AutoBuilder.buildAutoChooser("Mid-start")

        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def _attempt_vision_seed(self):
        """Attempt to seed the drivetrain pose and print the result."""
        success = self._vision.seed_drivetrain_pose()
        if success:
            print("Pose seeded successfully from Vision!")
        else:
            print("Pose seed failed: No valid vision targets in view.")

    def shooter_speed_change(self, speed_change: float):
        self.TARGET_SHOOTER_SPEED = self.TARGET_SHOOTER_SPEED - speed_change

        # Clamp speeds
        if self.TARGET_SHOOTER_SPEED > SHOOTER_MAX_RPS:
            self.TARGET_SHOOTER_SPEED = SHOOTER_MAX_RPS
        elif self.TARGET_SHOOTER_SPEED < SHOOTER_MIN_RPS:
            self.TARGET_SHOOTER_SPEED = SHOOTER_MIN_RPS

        print(f"shooter_speed_change +: {self.TARGET_SHOOTER_SPEED} ({speed_change})")
        self._shooter.flywheel_spin(self.TARGET_SHOOTER_SPEED)
