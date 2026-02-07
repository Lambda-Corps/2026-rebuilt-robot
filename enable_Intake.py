from typing import Self
from enum import Enum
from phoenix6 import controls
from commands2 import InstantCommand, Subsystem, Command, cmd
import wpilib
from intake import Intake

class Enable_Intake(Command):
    def __init__(self, intake: Intake, enable_control : bool, reverse : bool ):
        super().__init__()
        self.enable_control = enable_control
        self._intake = intake
        self._reverse = reverse
        self.addRequirements(self._intake)  

    def initialize(self):
        self._intake.enable_intake(self.enable_control, self._reverse) 
        print (f" Enable_intake {self.enable_control}")

    def execute(self):
        pass

    def isFinished(self) -> bool:          # This command only runs one time
        return True

    def end(self, interrupted: bool):
        pass
