# from enum import Enum
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder, Timer
from phoenix5 import TalonSRX, TalonSRXConfiguration, ControlMode, TalonSRXControlMode
from phoenix6 import utils
import wpilib

#===(Hardware Notes)==============================================
'''
The climber is moved using an 775 motor controlled by a Talon SRX.

The climber DOES  NOT have limit switches at the top and bottom of range
motion to prevent damage.

A prototype limit switch is partially installed which could perform this 
function but is not complete.

GREAT care must be used when driving the limit switch or hardware DAMAGE will occur

'''
#================================================================


class Climber(Subsystem):
    """
    Class to control the single linear actuator used to lift the robot
    """

    def __init__(self):
        WAIT_MILLISECONDS_BEFORE_LIMITING = 1000 # (1 second = 1000 milliSeconds)
        CURRENT_LIMIT = 2 # Amps
        super().__init__()
        self.Climber_Motor: TalonSRX = TalonSRX(31, "" if utils.is_simulation() else "canivore1")
        self.Climber_Motor.configFactoryDefault()
        self.Climber_Motor.configPeakCurrentLimit(CURRENT_LIMIT, WAIT_MILLISECONDS_BEFORE_LIMITING)  # Lmit the current


    def drive_motor(self, speed: float):
        self.Climber_Motor.set(TalonSRXControlMode.PercentOutput, speed)


    def stop_motor(self) -> None:
        self.Climber_Motor.set(ControlMode.PercentOutput, 0)

    def periodic(self) -> None:
        # SmartDashboard.putNumber("Climber_Speed", self.Climber_Motor.getMotorOutputPercent)
        pass