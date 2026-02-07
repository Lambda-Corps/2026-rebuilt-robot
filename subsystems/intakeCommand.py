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
from intake import Intake

class ControlIntake(Command):
    def __init__(self, intake: Intake, speed: float, timeout = 0):
        super().__init__()

        self._speed = speed
        self._intake = intake
        self._timeout = timeout

        self._timer = Timer()
        self._timer.start()

        self.addRequirements(self._intake)  

    def initialize(self):
        self._timer.restart()

    def execute(self):
        self._intake.intake_speed_global_control()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        pass
