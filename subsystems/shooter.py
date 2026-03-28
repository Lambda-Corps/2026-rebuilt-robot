from enum import Enum
import wpilib
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder

from utils.logger import (
    log_debug,
    log_smartdashboard_number,
    log_smartdashboard_boolean,
)
from constants import SHOOTER_MIN_RPS, SHOOTER_MAX_RPS
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXConfigurator,
)
from phoenix6 import hardware, controls, signals, utils
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import (
    InvertedValue,
    NeutralModeValue,
    ForwardLimitValue,
    ReverseLimitValue,
)
from phoenix6.unmanaged import feed_enable
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals.spn_enums import (
    InvertedValue,
    NeutralModeValue,
    FeedbackSensorSourceValue,
)
from phoenix6 import StatusCode
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue
from phoenix6.controls import DutyCycleOut, VelocityVoltage
from phoenix6.unmanaged import feed_enable
from phoenix6.signal_logger import SignalLogger


class Shooter(Subsystem):

    MOTOR_SPEED_GLOBAL = 50.0

    def __init__(self):
        super().__init__()

        # SignalLogger.start()                 # Enable CTRE Motor Hoot logging.

        self._shooter_indexer: TalonFX = self.__configure_indexer()
        self._shooter_flywheel: TalonFX = self.__configure_flywheel()
        self.intake_duty_cycle_out = controls.DutyCycleOut(0.0)
        self.flywheel_velocity_voltage = controls.VelocityVoltage(0.0)
        self.indexer_velocity_voltage = controls.VelocityVoltage(0.0)
        self.counter = 0

        self.MOTOR_SPEED_GLOBAL = 50.0  # Initial speed
        self.INDEXER_SPEED_GLOBAL = 0.0

    def __configure_indexer(self) -> TalonFX:
        talon = TalonFX(21, "" if utils.is_simulation() else "canivore1")
        config: TalonFXConfiguration = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        config.slot0.k_v = 0.12
        config.slot0.k_s = -0.012
        config.slot0.k_p = 0.2
        config.slot0.k_i = 0  # leave for now
        config.slot0.k_d = 0  # leave for now
        talon.configurator.apply(config)

        return talon

    def __configure_flywheel(self) -> TalonFX:
        talon = TalonFX(20, "" if utils.is_simulation() else "canivore1")     # CAN Bus Address
        config: TalonFXConfiguration = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        config.slot0.k_v = 0.12
        config.slot0.k_s = -0.012
        config.slot0.k_p = 0.2
        config.slot0.k_i = 0  # leave for now
        config.slot0.k_d = 0  # leave for now
        talon.configurator.apply(config)
        return talon

    def flywheel_spin(self, speed) -> None:
        self.MOTOR_SPEED_GLOBAL = speed
        self.flywheel_velocity_voltage.velocity = self.MOTOR_SPEED_GLOBAL
        self._shooter_flywheel.set_control(self.flywheel_velocity_voltage)
        # print(f"Flywheel speed set: {self.MOTOR_SPEED_GLOBAL:6.2f}")
        wpilib.SmartDashboard.putNumber("Flywheel RPS Requested", self.MOTOR_SPEED_GLOBAL)

    def periodic(self):
        dashboard_rps = wpilib.SmartDashboard.getNumber("Flywheel RPS Requested", self.MOTOR_SPEED_GLOBAL)
        if abs(dashboard_rps - self.MOTOR_SPEED_GLOBAL) > 0.001:
            self.flywheel_spin(dashboard_rps)

        dash_indexer_rps = wpilib.SmartDashboard.getNumber("Indexer RPS Requested", self.INDEXER_SPEED_GLOBAL)
        if abs(dash_indexer_rps - self.INDEXER_SPEED_GLOBAL) > 0.001:
            self.indexer_spin(dash_indexer_rps)

        rotor_velocity = self._shooter_flywheel.get_rotor_velocity()     # Get the flywheel speed
        rotor_velocity.refresh()
        velocity_value = rotor_velocity.value
        # print(f"Flywheel Speed target: {self.MOTOR_SPEED_GLOBAL:6.2}  actual_velocity: {velocity_value:6.2f}")
        wpilib.SmartDashboard.putNumber("Flywheel RPS Requested", self.MOTOR_SPEED_GLOBAL)
        wpilib.SmartDashboard.putNumber("FlyWheel RPS Actual: ", round(velocity_value, 2))

        indexer_vel = self._shooter_indexer.get_rotor_velocity()
        indexer_vel.refresh()
        wpilib.SmartDashboard.putNumber("Indexer RPS Requested", self.INDEXER_SPEED_GLOBAL)
        wpilib.SmartDashboard.putNumber("Indexer RPS Actual: ", round(indexer_vel.value, 2))

    def indexer_spin(self, indexer_spinspeed: float) -> None:
        self.INDEXER_SPEED_GLOBAL = indexer_spinspeed
        self.indexer_velocity_voltage.velocity = self.INDEXER_SPEED_GLOBAL
        self._shooter_indexer.set_control(self.indexer_velocity_voltage)
        wpilib.SmartDashboard.putNumber("Indexer RPS Requested", self.INDEXER_SPEED_GLOBAL)

    def change_speed_variable_function(self, speed_update: float) -> None:
        new_speed = self.MOTOR_SPEED_GLOBAL + speed_update
        if SHOOTER_MIN_RPS <= new_speed <= SHOOTER_MAX_RPS:
            self.MOTOR_SPEED_GLOBAL = new_speed
            self.flywheel_spin(self.MOTOR_SPEED_GLOBAL)
            print(f"Speed Changed {self.MOTOR_SPEED_GLOBAL:6.2f}")
        elif new_speed <= SHOOTER_MIN_RPS:
            self.MOTOR_SPEED_GLOBAL = SHOOTER_MIN_RPS
            print("Lower Limit")
        elif new_speed >= SHOOTER_MAX_RPS:
            self.MOTOR_SPEED_GLOBAL = SHOOTER_MAX_RPS
            print("Upper Limit")

    def is_shooter_spinning(self, thresholdPercent) -> bool :
        rotor_velocity = self._shooter_flywheel.get_rotor_velocity()
        # rotor_velocity.refresh()
        currentSpeed=-rotor_velocity.value
        # return True
        if currentSpeed > thresholdPercent*self.MOTOR_SPEED_GLOBAL:
            return True 
        else:return False
