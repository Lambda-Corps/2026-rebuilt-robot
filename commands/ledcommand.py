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
        # self.led.rainbow()
        if (not self.intake.is_intake_spinning()) and (not self.shooter.is_shooter_spinning()):
            self.led.red()
        elif (not self.intake.is_intake_spinning()) and (self.shooter.is_shooter_spinning()):
            self.led.blue()
        elif (self.intake.is_intake_spinning()) and (not self.shooter.is_shooter_spinning()):
            self.led.green()  
        elif (self.intake.is_intake_spinning()) and (self.shooter.is_shooter_spinning()):
            self.led.purple()      
               



    def end(self, interrupted: bool) -> None:
       pass       #  This function is not being used.

    def isFinished(self) -> bool:
       return False


