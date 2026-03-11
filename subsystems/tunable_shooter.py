import wpilib
from commands2 import Subsystem, Command, SequentialCommandGroup, cmd, WaitUntilCommand
from wpilib import SmartDashboard
from wpimath.filter import Debouncer

from phoenix6.configs import (
    TalonFXConfiguration,
)
from phoenix6 import utils
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals.spn_enums import (
    NeutralModeValue,
)
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals.spn_enums import (
    NeutralModeValue,
)
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals.spn_enums import NeutralModeValue
from phoenix6.controls import DutyCycleOut, VelocityVoltage, NeutralOut


class TunableShooter(Subsystem):
    def __init__(self):
        super().__init__()

        # SignalLogger.start()                 # Enable CTRE Motor Hoot logging.

        self._shooter_indexer: TalonFX = self.__configure_indexer()
        self._shooter_flywheel: TalonFX = self.__configure_flywheel()

        self.velocity_control = VelocityVoltage(0) # used to set the motor
        self.stop_motor_control = NeutralOut(0)

        self.index_duty_cycle = DutyCycleOut(0)
        self.indexer_duty_cycle_out = DutyCycleOut(0.0)

        # Debouncer used to make sure the flywheel is at speed
        # This requires the condition to be True for 0.06 seconds (3 scheduler loops)
        self.at_speed_debouncer = Debouncer(0.06, Debouncer.DebounceType.kRising)
        
        # Default tuning values
        SmartDashboard.putNumber("s_k_p", 0.0)
        SmartDashboard.putNumber("Shooter/kP", 0.1)
        SmartDashboard.putNumber("Shooter/kV", 0.12)
        SmartDashboard.putNumber("Shooter/kS", 0.0)
        SmartDashboard.putNumber("Shooter/Target RPS", 50.0)
        SmartDashboard.putNumber("Shooter/Tolerance", 2.0) # How close is "at speed"?
        SmartDashboard.putNumber("Shooter/IndexerSpeed", 0.5 ) # Needs to be measured first
        self._target_rps = -30
        self._target_rps_tolerance = 2
        self._indexer_speed = 0.0

        self.motor_speed_global = 0

        SmartDashboard.putData("ShooterScheduler", self)

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

        # To control the motor in closed-loop mode we need to set the slot configurations 
        # for built-in PID in the motor controllers
        # We need to test and tune to find the following values:
        # Ks = Voltage to overcome static friction, to find it use phoenix tuner to gradually
        #      increase the motor output until the wheels spin
        # Kv = This is the feedforward, the voltage necessary to get to the desired speed. 
        #      The math sort of works out by calculating 
        #      the reciprocal of the motors theoretical max speed.  Math equation is 
        #               Battery Voltage (12V)             12
        #       kv =  -------------------------    =    -------  = .12
        #                  Max Motor RPS                  100
        #      To find it, with no ball in the flywheel, use Phoenix tuner to measure the speed
        #      and convert to RPS -- For example, 6000 RPM = 100 RPS
        # Kp = This is the feedback, the voltage you want to add to recover from a certain error.
        #      For example, if our desired speed is 50 rps (at 6 volts), then we maybe want to 
        #      apply an extra 1 volt for every 5 rps we're behind.  The math would be:
        #                 1 volt                1
        #       kp =  --------------      =   -----  = .2
        #                 5 rps                 5
        #
        # These are theoretical starting values, they should be estimated with the math but then
        # empirically measured and tuned.
        # This initial configuration in this code will set them all to zero for safety.  We will
        # use Smart Dashboard to tune these and update this configuration.
        config.slot0.k_v = -0.114
        config.slot0.k_s = -0.0148
        config.slot0.k_p = 0
        config.slot0.k_i = 0  # leave for now
        config.slot0.k_d = 0  # leave for now
        talon.configurator.apply(config)
        return talon

    def flywheel_spin(self, speed) -> None:
        self.flywheel_duty_cycle_out.output = speed
        self._shooter_flywheel.set_control(self.flywheel_duty_cycle_out)
        print(f"Flywheel speed set: {speed}")

    def periodic(self):
        rotor_velocity = self._shooter_flywheel.get_rotor_velocity(refresh=True)     # Get the flywheel speed
        SmartDashboard.putNumber("FlyWheel Velocity: ", rotor_velocity.value_as_double)

    def index_with_speed(self, indexer_spinspeed: float) -> None:
        self.indexer_duty_cycle_out.output = indexer_spinspeed
        self._shooter_indexer.set_control(self.indexer_duty_cycle_out)
        wpilib.SmartDashboard.putNumber("Intake Speed: ", indexer_spinspeed)

    def index_run(self) -> None:
        # You must call set_control to actually make the motor move!
        self._shooter_indexer.set_control(self.index_duty_cycle.with_output(self._indexer_speed))

    def index_stop(self) -> None:
        self.index_duty_cycle.with_output(0)

    def change_speed_variable_function(self, speed_update: float) -> None:
        if (self.motor_speed_global > -1) and (self.motor_speed_global < 1):
            self.motor_speed_global = self.motor_speed_global + speed_update
            print("Speed Changed")
            print(self.motor_speed_global)
        elif self.motor_speed_global <= -1:
            self.motor_speed_global = -0.95
            print("Lower Limit")
        elif self.motor_speed_global >= 1:
            self.motor_speed_global = 0.95
            print("Upper Limit")
        

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

    def update_config_from_dashboard(self):
        cfg = TalonFXConfiguration()
        cfg.slot0.k_p = SmartDashboard.getNumber("Shooter/kP", 0.0)
        kp = SmartDashboard.getNumber("s_k_p", 0.0)
        cfg.slot0.k_v = SmartDashboard.getNumber("Shooter/kV", 0.0)
        cfg.slot0.k_s = SmartDashboard.getNumber("Shooter/kS", 0.0)

        print (f"cfg.slot0.k_p: {cfg.slot0.k_p}  ", end='')
        print (f"cfg.slot0.k_v: {cfg.slot0.k_v}  ", end='')
        print (f"cfg.slot0.k_s: {cfg.slot0.k_s} ")
        print ("===================================")
        
        # Use the correct motor reference
        self._shooter_flywheel.configurator.apply(cfg, timeout_seconds=.2)

        self._target_rps = SmartDashboard.getNumber("Shooter/Target RPS", 0.0)
        self._target_rps_tolerance = SmartDashboard.getNumber("Shooter/Tolerance", 0.0)
        self._indexer_speed = SmartDashboard.getNumber("Shooter/IndexerSpeed", 0.0)
    
    def is_at_speed(self) -> bool:
        actual = self._shooter_flywheel.get_velocity().value

        # Check the raw condition
        currently_at_speed = abs(self._target_rps - actual) < self._target_rps_tolerance
        
        # Apply the debouncer
        # This returns True only if currently_at_speed has been True 
        # for the duration defined in __init__
        return self.at_speed_debouncer.calculate(currently_at_speed)
    
    def run_shooter(self) -> None:
        self._shooter_flywheel.set_control(self.velocity_control.with_velocity(self._target_rps))

    def stop_shooter(self) -> None:
        self._shooter_flywheel.set_control(self.stop_motor_control)

    def stop_motors(self) -> None:
        self.stop_shooter()
        self.index_stop()

    def run_shoot_sequence(self) -> Command:
        return SequentialCommandGroup(
                # 1. Start the motor
                cmd.runOnce(lambda: self.run_shooter, self),
                
                # 2. Wait until the velocity is within tolerance
                WaitUntilCommand(self.is_at_speed),
                
                # 3. Start the indexer (replace with your actual indexer motor call)
                cmd.runOnce(lambda: self.index_run(), self) 
        )
    
    def indexer_spin(self, indexer_spinspeed: float) -> None:
        self.indexer_duty_cycle_out.output = indexer_spinspeed
        self._shooter_indexer.set_control(self.indexer_duty_cycle_out)
        wpilib.SmartDashboard.putNumber("Intake Speed: ", indexer_spinspeed)