from enum import Enum
import wpilib
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder
from wpimath.system.plant import DCMotor
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXConfigurator,
)
from phoenix6 import hardware, controls, signals, utils
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
        self.intake_velocity_voltage = controls.VelocityVoltage(0.0)
        self.motor_speed_global = 0.0  # Initial speed
        self.intake_enabled = False
        self.intake_reversed = False

    def __configure_intake(self) -> TalonFX:
        talon = TalonFX(30, "" if utils.is_simulation() else "canivore1")
        config: TalonFXConfiguration = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        config.slot0.k_v = 0.18
        config.slot0.k_s = -0.012
        config.slot0.k_p = 0.2
        config.slot0.k_i = 0
        config.slot0.k_d = 0
        talon.configurator.apply(config)
        return talon
    
    def periodic(self) -> None:
        dashboard_rps = wpilib.SmartDashboard.getNumber("Intake RPS Requested", self.motor_speed_global)
        
        # Determine current requested sign since it could be negative if reversed
        current_requested = self.motor_speed_global * (-1 if self.intake_reversed else 1)
        if not self.intake_enabled:
            current_requested = 0.0
            
        if abs(dashboard_rps - current_requested) > 0.001:
            if dashboard_rps < 0:
                self.enable_intake(abs(dashboard_rps), True)
            elif dashboard_rps > 0:
                self.enable_intake(dashboard_rps, False)
            else:
                self.enable_intake(0.0, False)

        rotor_velocity = self._intake_motor.get_rotor_velocity()     # Get the Intake speed
        rotor_velocity.refresh()
        velocity_value = rotor_velocity.value
        
        current_out = self.motor_speed_global * (-1 if self.intake_reversed else 1) if self.intake_enabled else 0.0
        wpilib.SmartDashboard.putNumber("Intake RPS Requested", current_out)
        wpilib.SmartDashboard.putNumber("Intake RPS Actual: ", round(velocity_value, 2))

    def intake_speed_global_control(self) -> None:
        #print("Global Control Ran~")  
        if (self.intake_enabled and not self.intake_reversed):                                         # Global control of intake speed
            self.intake_velocity_voltage.velocity = self.motor_speed_global    # Speed set by global variable
            self._intake_motor.set_control(self.intake_velocity_voltage)
        elif (self.intake_enabled and self.intake_reversed):
            self.intake_velocity_voltage.velocity = self.motor_speed_global * -1
            self._intake_motor.set_control(self.intake_velocity_voltage)
        else:
            self.intake_velocity_voltage.velocity = 0           # Speed set by global variable
            self._intake_motor.set_control(self.intake_velocity_voltage)

    # def change_speed_variable_function(self, speed_update : float) -> None:
    #     if ((self.motor_speed_global > -1 ) and (self.motor_speed_global < 1)):
    #         self.motor_speed_global = self.motor_speed_global + speed_update
    #     #print (f">>>>> self.motor_speed_global {self.motor_speed_global}   Subsystem")

    def enable_intake(self, speed, reverse):
        self.intake_enabled  = speed != 0
        if speed != 0:
            self.motor_speed_global = speed
        self.intake_reversed = reverse
        #print (f">>>> self.intake_enabled {self.intake_enabled}")
        self.intake_speed_global_control()


        # added comment (testing source control)
        # Added Second Change comment
        # Addeing third Comments
        # Adding forth Comment

    def is_intake_spinning(self) -> bool :
        rotor_velocity = self._intake_motor.get_rotor_velocity()
        # rotor_velocity.refresh()
        velocity_value = rotor_velocity.value
        # print (velocity_value) 
        if velocity_value > 20:
            return True 
        else:
            return False
        