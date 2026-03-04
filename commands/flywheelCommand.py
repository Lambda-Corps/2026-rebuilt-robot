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

from subsystems.shooter_subsystem import ShooterSubsystem


class ControlFlywheel(Command):
    def __init__(self, sub: ShooterSubsystem, speed: float):
        super().__init__()
        self.addRequirements(sub)

        self._speed = speed
        self._FlywheelSub = sub

        print(f"ControlFlywheel Command Created with speed: {speed}")
        self._FlywheelSub.set_shooter_speed(speed)

    def initialize(self):
        pass

    def execute(self):
        self._FlywheelSub.set_shooter_speed(self._speed)
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        pass