from typing import Self
from enum import Enum
from phoenix6 import controls
from commands2 import InstantCommand, Subsystem, Command, cmd
# from phoenix5 import (
#     TalonSRX,
#     TalonSRXControlMode,
#     TalonSRXConfiguration,
#     LimitSwitchSource,
#     Faults,
#     LimitSwitchNormal,
# )
import wpilib
#===========================================================
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXConfigurator,
)


from phoenix6 import hardware, controls, signals
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, ForwardLimitValue, ReverseLimitValue
from phoenix6.unmanaged import feed_enable
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals.spn_enums import (InvertedValue, NeutralModeValue, FeedbackSensorSourceValue)
from phoenix6 import StatusCode
from wpilib import SmartDashboard, AnalogInput, RobotBase, Timer
from subsystems.tunable_shooter import TunableShooter

class velocityControlFlywheel(Command):
    def __init__(self, sub: TunableShooter):
        super().__init__()

        self._Flywheel = sub

        self.addRequirements(self._Flywheel)  

    def initialize(self):
        self._Flywheel.update_config_from_dashboard()
        print("Updated config")

    def execute(self):
        self._Flywheel.run_shooter()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._Flywheel.stop_shooter()


class StopFlywheel(Command):
    def __init__(self, sub: TunableShooter):
        super().__init__()

        self._Flywheel = sub

        self.addRequirements(self._Flywheel)  

    def initialize(self):
        self._Flywheel.stop_motors()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        self._Flywheel.stop_shooter()