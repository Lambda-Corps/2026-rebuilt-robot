import wpilib
from commands2 import Command
from subsystems.ledsubsystem import LEDSubsystem
from subsystems.shooter import Shooter
from subsystems.intake import Intake

class LEDCommand(Command):

    def __init__(self, led: LEDSubsystem, shooter: Shooter, Intake: Intake ) -> None:
        super().__init__()

        self.led = led
        self.shooter = shooter
        self.intake = Intake
      
        self.addRequirements(led)

    def initialize(self) -> None:
        pass       #  This function is not being used.

    def execute(self) -> None:
        self.led.rainbow()

    def end(self, interrupted: bool) -> None:
       pass       #  This function is not being used.

    def isFinished(self) -> bool:
       return False


