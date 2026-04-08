from commands2 import Command
from subsystems.intake import Intake
import wpilib

from constants import (
    INTAKE_SPEED_DEFAULT,
    INTAKE_AGGITATOR_CW_DEGREES,
    INTAKE_AGGITATOR_CCW_DEGREES
)

class IntakeOscillate(Command):
    """
    Rotates the intake 180° CW then 90° CCW, cycling until interrupted.
    Only runs when motor_speed_global != 0.
    """

    def __init__(self, intake: Intake):
        super().__init__()
        self._intake = intake
        self._timer = wpilib.Timer()
        self._phase = 0          # 0 = CW, 1 = CCW
        self._phase_duration = 0.0
        self._initial_reverse = False
        self.addRequirements(intake)

    def initialize(self):
        self._initial_reverse = self._intake.intake_reversed
        self._timer.reset()
        self._timer.start()
        self._start_phase(0)

    def _start_phase(self, phase: int):
        self._phase = phase
        speed_abs = abs(self._intake.motor_speed_global)
        if speed_abs == 0:
            self._phase_duration = float('inf')  # stall — do nothing
            return
        
        degrees = INTAKE_AGGITATOR_CW_DEGREES if phase == 0 else INTAKE_AGGITATOR_CCW_DEGREES
        
        # Duration is degrees / 360 / absolute speed (Rotations/Sec)
        # Assuming speed is roughly in Rotations per Second (RPS)
        self._phase_duration = (degrees / 360.0) / speed_abs * 20
        self._timer.reset()

    def execute(self):
        current_speed = self._intake.motor_speed_global
        speed_abs = abs(current_speed)
        
        if speed_abs == 0:
            self._intake.enable_intake(INTAKE_SPEED_DEFAULT / 4, False)
            return

        if self._timer.get() >= self._phase_duration:
            # Advance to next phase
            self._start_phase(1 - self._phase)

        # Phase 0 is normal (False), Phase 1 is reverse (True)
        phase_reverse = (self._phase == 1)
        
        # XOR with the initial reverse state so it works cleanly if they started in reversed mode
        actual_reverse = phase_reverse ^ self._initial_reverse
        
        self._intake.enable_intake(current_speed, actual_reverse)

    def isFinished(self) -> bool:
        return False  # Runs until interrupted (whileTrue)

    def end(self, interrupted: bool):
        self._timer.stop()
        # Restore the initial reverse state with the original speed
        self._intake.enable_intake(self._intake.motor_speed_global, self._initial_reverse)
