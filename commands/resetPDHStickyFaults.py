from commands2 import Command
from subsystems.shooter import Shooter

class ResetPDHStickyFaults(Command):
    def __init__(self, sub: Shooter):
        super().__init__()
        self._Flywheel = sub
        self.addRequirements(self._Flywheel)  

    def initialize(self):
        self._Flywheel.clearpowerhubstickyfaults()
        print("Reset PDH Sticky Faluts")

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        pass