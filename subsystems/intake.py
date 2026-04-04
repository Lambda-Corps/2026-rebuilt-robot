from enum import Enum
import wpilib
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder
from wpimath.system.plant import DCMotor, LinearSystemId
from wpilib.simulation import FlywheelSim
import math
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
        self.intake_velocity_voltage = controls.VelocityVoltage(0.0)  # Instantiate speed control object
        self.motor_speed_global = 0.0  # Initial speed
        self.intake_enabled = False
        self.intake_reversed = False

        if utils.is_simulation():
            # Using a single Falcon 500, 1:1 gear ratio, and an extremely low MOI of 0.0001 kg*m^2 for millisecond-level responsiveness
            intake_gearbox = DCMotor.falcon500(1)
            intake_plant = LinearSystemId.flywheelSystem(intake_gearbox, 0.0001, 1.0)
            self.intake_sim = FlywheelSim(intake_plant, intake_gearbox)

    def __configure_intake(self) -> TalonFX:
        talon = TalonFX(30, "" if utils.is_simulation() else "canivore1")
        config: TalonFXConfiguration = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        config.slot0.k_v = 0.12
        config.slot0.k_s = 0.0
        config.slot0.k_p = 0.2
        talon.configurator.apply(config)
        return talon
    
    def periodic(self) -> None:
        # Get the Velocity of the wheel in Rotations per second    
        rotor_velocity = self._intake_motor.get_rotor_velocity()     # Get the Actual Intake speed
        rotor_velocity.refresh()
        velocity_value = rotor_velocity.value
        wpilib.SmartDashboard.putNumber("Intake Speed Actual", round(velocity_value, 1))
        wpilib.SmartDashboard.putNumber("Intake Speed Requested", round(self.intake_velocity_voltage.velocity, 1))

    def simulationPeriodic(self) -> None:
        # 1. Get the applied motor voltage from the simulated TalonFX
        motor_voltage = self._intake_motor.sim_state.motor_voltage
        
        # 2. Feed it into our WPILib physics model (assume standard 20ms loop time)
        self.intake_sim.setInputVoltage(motor_voltage)
        self.intake_sim.update(0.02)
        
        # 3. Read the simulated velocity and pass it back to the TalonFX sim state
        # WPILib calculates velocity in Rads/Sec, Phoenix 6 expects Rotations/Sec
        sim_velocity_rps = self.intake_sim.getAngularVelocity() / (2 * math.pi)
        self._intake_motor.sim_state.set_rotor_velocity(sim_velocity_rps)

    def intake_speed_global_control(self) -> None:
        # Global control of intake speed via VelocityVoltage 
        if self.intake_enabled and not self.intake_reversed:
            self.intake_velocity_voltage.velocity = self.motor_speed_global
        elif self.intake_enabled and self.intake_reversed:
            self.intake_velocity_voltage.velocity = -self.motor_speed_global
        else:
            self.intake_velocity_voltage.velocity = 0.0
            
        self._intake_motor.set_control(self.intake_velocity_voltage)

    # def change_speed_variable_function(self, speed_update : float) -> None:
    #     if ((self.motor_speed_global > -1 ) and (self.motor_speed_global < 1)):
    #         self.motor_speed_global = self.motor_speed_global + speed_update
    #     #print (f">>>>> self.motor_speed_global {self.motor_speed_global}   Subsystem")

    def enable_intake(self, speed: float, reverse: bool):
        self.motor_speed_global = speed
        self.intake_enabled = True if speed != 0.0 else False
        self.intake_reversed = reverse
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
        