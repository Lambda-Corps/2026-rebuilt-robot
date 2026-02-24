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
from subsystems.intake import Intake

class ControlIntake(Command):
    def __init__(self, intake: Intake, speed: float, reverse: bool):
        super().__init__()

        self._speed = speed
        self._intake = intake
        self._reverse = reverse

        self.addRequirements(self._intake)  

    def initialize(self):
        pass

    def execute(self):
        self._intake.enable_intake(self._speed, self._reverse)

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        pass
