from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder, Timer
from phoenix5 import TalonSRX, TalonSRXConfiguration, ControlMode, TalonSRXControlMode
import wpilib
from subsystems.climber import Climber


class SetClimberSpeedandTime(Command):
    def __init__(self, climber: Climber, speed: float, runseconds: float):
        self._climber = climber
        self.speed = speed
        self.runseconds = runseconds
        self._timer = Timer()
        self._timer.start()
        self.addRequirements(self._climber)

    def initialize(self):
        self._timer.restart()
        print ("Running Climber speed: ",self.speed, " for ", self.runseconds, "seconds at "
               , wpilib.Timer.getFPGATimestamp()  )

    def execute(self):
        self._climber.drive_motor(self.speed)
       
    def isFinished(self) -> bool:
        return self._timer.hasElapsed(self.runseconds)
    
    def end(self, interrupted: bool):
        self._climber.stop_motor()