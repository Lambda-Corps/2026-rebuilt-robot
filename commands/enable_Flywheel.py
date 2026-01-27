from typing import Self
from enum import Enum
from phoenix6 import controls
from commands2 import InstantCommand, Subsystem, Command, cmd
import wpilib
from subsystems.shooter import Shooter


class Enable_flywheel(Command):
    def __init__(self, sub: Shooter, enable_control : bool ):
        super().__init__()

        self.enable_control = enable_control
        self._flywheel = sub

        self.addRequirements(self._flywheel)  

    def initialize(self):
        self._flywheel.enable_flywheel(self.enable_control) 
        print (f" Enable_flywheel {self.enable_control}")

    def execute(self):
        pass


    def isFinished(self) -> bool:          # This command only runs one time
        return True

    def end(self, interrupted: bool):
        pass
