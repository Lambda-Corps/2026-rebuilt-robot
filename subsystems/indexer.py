from commands2 import Subsystem
import wpilib
from phoenix6 import controls, utils
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor, LinearSystemId
import math
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals.spn_enums import NeutralModeValue

class Indexer(Subsystem):

    def __init__(self):
        super().__init__()

        self._shooter_indexer: TalonFX = self.__configure_indexer()
        self.indexer_velocity_voltage = controls.VelocityVoltage(0.0)

        self.INDEXER_SPEED_GLOBAL = 0.0
        self._indexer_reversed = False

        if utils.is_simulation():
            indexer_gearbox = DCMotor.falcon500(1)
            indexer_plant = LinearSystemId.flywheelSystem(indexer_gearbox, 0.001, 1.0)
            self.indexer_sim = FlywheelSim(indexer_plant, indexer_gearbox)

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

    def periodic(self):
        indexer_vel = self._shooter_indexer.get_rotor_velocity()
        indexer_vel.refresh()
        wpilib.SmartDashboard.putNumber("Indexer RPS Requested", round(self.INDEXER_SPEED_GLOBAL, 1))
        wpilib.SmartDashboard.putNumber("Indexer RPS Actual", round(indexer_vel.value, 1))

    def simulationPeriodic(self) -> None:
        motor_voltage = self._shooter_indexer.sim_state.motor_voltage
        self.indexer_sim.setInputVoltage(motor_voltage)
        self.indexer_sim.update(0.02)
        sim_velocity_rps = self.indexer_sim.getAngularVelocity() / (2 * math.pi)
        self._shooter_indexer.sim_state.set_rotor_velocity(sim_velocity_rps)

    def set_indexer_reversed(self, reverse: bool) -> None:
        self._indexer_reversed = reverse
        self.indexer_spin(self.INDEXER_SPEED_GLOBAL)

    def indexer_spin(self, indexer_spinspeed: float) -> None:
        self.INDEXER_SPEED_GLOBAL = indexer_spinspeed
        actual_speed = -indexer_spinspeed if getattr(self, '_indexer_reversed', False) else indexer_spinspeed
        self.indexer_velocity_voltage.velocity = actual_speed
        self._shooter_indexer.set_control(self.indexer_velocity_voltage)
        wpilib.SmartDashboard.putNumber("Indexer Spin", actual_speed)
