import time
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
from wpilib import TimedRobot, Timer
from subsystems.shooter import Shooter

class FireInTheHole(Command):
    def __init__(self, sub: Shooter, speed: float, timeout = float):
        super().__init__()

        self._timer = Timer()
        self._speed = speed
        self._INDEXER = sub
        self._FLYWHEEL = sub
        self._timeout = timeout

        self._timer.start()

        self.addRequirements(self._INDEXER)  

    def initialize(self):
        self._timer.restart()

    def execute(self):
        self._FLYWHEEL.flywheel_spin(self._speed)

    def isFinished(self) -> bool:
        
        return self._timer.hasElapsed(self._timeout)

    def end(self, interrupted: bool):
        self._FLYWHEEL.flywheel_spin(0)
        self._INDEXER.indexer_spin(self._speed)
        self._INDEXER.indexer_spin(0)