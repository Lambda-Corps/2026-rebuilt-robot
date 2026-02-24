from enum import Enum
import wpilib
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder

from utils.logger import (
    log_debug,
    log_smartdashboard_number,
    log_smartdashboard_boolean,
)
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
    def __init__(self):
        super().__init__()

        # SignalLogger.start()                 # Enable CTRE Motor Hoot logging.

        self._shooter_indexer: TalonFX = self.__configure_indexer()
        self._shooter_flywheel: TalonFX = self.__configure_flywheel()
        self.flywheel_duty_cycle_out = controls.DutyCycleOut(0.0)
        self.indexer_duty_cycle_out = controls.DutyCycleOut(0.0)
        self.counter = 0

        self.motor_speed_global = 0.5  # Initial speed
        self.newMotorSpeed = self.motor_speed_global

    def __configure_indexer(self) -> TalonFX:
        talon = TalonFX(21, "" if utils.is_simulation() else "canivore1")
        config: TalonFXConfiguration = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        talon.configurator.apply(config)

        return talon

    def __configure_flywheel(self) -> TalonFX:
        talon = TalonFX(20, "" if utils.is_simulation() else "canivore1")     # CAN Bus Address
        config: TalonFXConfiguration = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        talon.configurator.apply(config)
        return talon

    def flywheel_spin(self, flywheel_spinspeed: float) -> None:
        self.motor_speed_global=flywheel_spinspeed
        self.flywheel_duty_cycle_out.output = self.motor_speed_global
        self._shooter_flywheel.set_control(self.flywheel_duty_cycle_out)
            

        # rotor_velocity = self._shooter_flywheel.get_rotor_velocity()
        # rotor_velocity.refresh()
        # velocity_value = rotor_velocity.value
        # print(f"velocity_value: {velocity_value:6.2f}")


    # def flywheel_spin_global_control(self) -> None:         
    #     self.flywheel_enabled = True  # Speed not being controlled by parameter
    #     if (self.flywheel_enabled):                                       # Global control of Flywheel
    #         self.counter = self.counter  + 1

    #         self.flywheel_duty_cycle_out.output = self.motor_speed_global           # Speed set by global variable
    #         self._shooter_flywheel.set_control(self.flywheel_duty_cycle_out)

    #     else:
    #         self.flywheel_duty_cycle_out.output = 0
    #         self._shooter_flywheel.set_control(self.flywheel_duty_cycle_out)

    #     # Get the Velocity of the wheel in Rotations per second    
    #     


    def periodic(self):
        rotor_velocity = self._shooter_flywheel.get_rotor_velocity()     # Get the flywheel speed
        rotor_velocity.refresh()
        velocity_value = rotor_velocity.value
        #print(f"Global Speed: {self.motor_speed_global:6.2}       velocity_value: {velocity_value:6.2f}")
        wpilib.SmartDashboard.putNumber("FlyWheel Velocity: ", velocity_value)
        wpilib.SmartDashboard.putNumber("Wanted Flywheel Velocity: ", self.motor_speed_global)

    # def indexer_spin(self,indexer_spinspeed: float) -> None:
    #     # Get the Velocity of the wheel in Rotations per second
    #     rotor_velocity = (
    #         self._shooter_flywheel.get_rotor_velocity()
    #     )  # Get the flywheel speed
    #     rotor_velocity.refresh()
    #     velocity_value = rotor_velocity.value
    #     # print(f"Counter: {self.counter}     Global Speed: {self.motor_speed_global:6.2}       velocity_value: {velocity_value:6.2f}")

    #     log_smartdashboard_number(
    #         "FlyWheel Speed: ",
    #         self.motor_speed_global,
    #         min_verbosity=10,
    #         update_frequency=10,
    #     )
    #     log_smartdashboard_number(
    #         "FlyWheel Velocity: ", velocity_value, min_verbosity=10, update_frequency=10
    #     )
    #     log_smartdashboard_boolean(
    #         "Flywheel Enable: ",
    #         self.flywheel_enabled,
    #         min_verbosity=10,
    #         update_frequency=10,
    #     )

    def indexer_spin(self, indexer_spinspeed: float) -> None:
        self.indexer_duty_cycle_out.output = indexer_spinspeed
        self._shooter_indexer.set_control(self.indexer_duty_cycle_out)
        wpilib.SmartDashboard.putNumber("Indexer Speed: ", indexer_spinspeed)

    def change_speed_variable_function(self, speed_update : float) -> None:
        
        if ((self.motor_speed_global > -1 ) and (self.motor_speed_global < 1)):
            self.motor_speed_global = self.motor_speed_global + speed_update
        elif self.motor_speed_global == -1:
            self.motor_speed_global = -0.95
        elif self.motor_speed_global == 1:
            self.motor_speed_global = 0.95
        

        #print (f">>>>> self.motor_speed_global {self.motor_speed_global}   Subsystem")

    # def enable_flywheel(self, enable, flywheel_spinspeed):
    #     self.flywheel_enabled  = enable
    #     self.flywheel_spin(flywheel_spinspeed)

    def is_shooter_spinning(self, thresholdPercent) -> bool :
        rotor_velocity = self._shooter_flywheel.get_rotor_velocity()
        # rotor_velocity.refresh()
        currentSpeed=-rotor_velocity.value
        # return True
        if currentSpeed > thresholdPercent*self.motor_speed_global:
            return True 
        else:return False
