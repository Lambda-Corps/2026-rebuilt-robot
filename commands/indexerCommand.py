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
from subsystems.shooter import Shooter
from subsystems.ledsubsystem import LEDSubsystem

class ControlIndexer(Command):
    def __init__(self, sub: Shooter, speed: float):
        super().__init__()

        self._speed = speed
        self._ShooterSubSys = sub

        self.addRequirements(self._ShooterSubSys)  

    def initialize(self):
        pass

    def execute(self):
        # if (self._ShooterSubSys.is_shooter_spinning(0.3)):
        self._ShooterSubSys.indexer_spin(self._speed)
        # else:
        #     self._ShooterSubSys.flywheel_spin(0.5)
        #     print("Flywheel getting ready.")


    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        pass