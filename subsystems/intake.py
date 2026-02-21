from enum import Enum
import wpilib
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder
from wpimath.system.plant import DCMotor
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXConfigurator,
)
from phoenix6 import hardware, controls, signals
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, ForwardLimitValue, ReverseLimitValue
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals.spn_enums import (InvertedValue, NeutralModeValue, FeedbackSensorSourceValue)
from phoenix6 import StatusCode
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue
from phoenix6.controls import DutyCycleOut, VelocityVoltage
from phoenix6.signal_logger import SignalLogger

class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self._intake_motor: TalonFX = self.__configure_intake()  # Instantiate motor
        self.intake_duty_cycle_out = controls.DutyCycleOut(0.0)  # Instantiate speed control object
        self.motor_speed_global = 1  # Initial speed
        self.intake_enabled = False
        self.intake_reversed = False

    def __configure_intake(self) -> TalonFX:
        talon = TalonFX(30, "canivore1")
        config: TalonFXConfiguration = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        talon.configurator.apply(config)
        return talon
    
    def intake_speed_global_control(self) -> None:
        #print("Global Control Ran~")  
        if (self.intake_enabled and not self.intake_reversed):                                         # Global control of intake speed
            self.intake_duty_cycle_out.output = self.motor_speed_global    # Speed set by global variable
            self._intake_motor.set_control(self.intake_duty_cycle_out)
        elif (self.intake_enabled and self.intake_reversed):
            self.intake_duty_cycle_out.output = self.motor_speed_global * -1
            self._intake_motor.set_control(self.intake_duty_cycle_out)
        else:
            self.intake_duty_cycle_out.output = 0           # Speed set by global variable
            self._intake_motor.set_control(self.intake_duty_cycle_out)

        # Get the Velocity of the wheel in Rotations per second    
        rotor_velocity = self._intake_motor.get_rotor_velocity()     # Get the Intake speed
        rotor_velocity.refresh()
        velocity_value = rotor_velocity.value

        wpilib.SmartDashboard.putNumber("Intake Speed: ", self.motor_speed_global)
        wpilib.SmartDashboard.putNumber("Intake Velocity: ", velocity_value)

    # def change_speed_variable_function(self, speed_update : float) -> None:
    #     if ((self.motor_speed_global > -1 ) and (self.motor_speed_global < 1)):
    #         self.motor_speed_global = self.motor_speed_global + speed_update
    #     #print (f">>>>> self.motor_speed_global {self.motor_speed_global}   Subsystem")

    def enable_intake(self, enable, reverse):
        self.intake_enabled  = enable
        self.intake_reversed = reverse
        #print (f">>>> self.intake_enabled {self.intake_enabled}")
        self.intake_speed_global_control()


        # added comment (testing source control)
        # Added Second Change comment
        # Addeing third Comments
        # Adding forth Comment

    def is_intake_spinning(self) -> bool :
        rotor_velocity = self._intake_motor.get_rotor_velocity()
        rotor_velocity.refresh()
        velocity_value = rotor_velocity.value
        # return False  
        if velocity_value > 20:
            return True 
        else:return False