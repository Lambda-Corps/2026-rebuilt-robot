from typing import Self
from enum import Enum
from phoenix6 import controls
from commands2 import InstantCommand, Subsystem, Command, cmd
import wpilib
from subsystems.shooter import Shooter


class Update_Speed_Variable(Command):
    def __init__(self, sub: Shooter, speed_Change: float, ):
        super().__init__()

        self.speed_Change = speed_Change
        self._flywheel = sub

        self.addRequirements(self._flywheel)  

    def initialize(self):
        self._flywheel.change_speed_variable_function(self.speed_Change) 
        print (f">>>>>> speed {self.speed_Change}   Command")

    def execute(self):
        pass


    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        pass
