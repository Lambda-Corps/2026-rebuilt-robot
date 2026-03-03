"""ShooterSubsystem: Manages shooter and indexer motors.

Owns two TalonFX motors on the CANivore bus:
- Shooter motor (CAN ID 20): Launches game pieces at variable speed
- Indexer motor (CAN ID 21): Feeds game pieces into the shooter

Motor outputs are controlled via DutyCycleOut requests.
"""

from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut
from wpilib import SmartDashboard


class ShooterSubsystem(Subsystem):
    """Subsystem managing shooter and indexer TalonFX motors."""

    SHOOTER_CAN_ID = 20
    INDEXER_CAN_ID = 21

    def __init__(self) -> None:
        """Initialize hardware and control requests."""
        super().__init__()

        self._shooter_motor = TalonFX(self.SHOOTER_CAN_ID, "canivore1")
        self._indexer_motor = TalonFX(self.INDEXER_CAN_ID, "canivore1")

        # Pre-allocate control requests to avoid per-cycle allocation
        self._shooter_request = DutyCycleOut(0.0)
        self._indexer_request = DutyCycleOut(0.0)

        self._shooter_duty_cycle: float = 0.0
        self._indexer_duty_cycle: float = 0.0

    def periodic(self) -> None:
        """Publish telemetry every cycle."""
        SmartDashboard.putNumber("Shooter/ShooterDutyCycle", self._shooter_duty_cycle)
        SmartDashboard.putNumber("Shooter/IndexerDutyCycle", self._indexer_duty_cycle)

    def set_shooter_speed(self, duty_cycle: float) -> None:
        """Set shooter motor duty cycle (-1.0 to 1.0)."""
        self._shooter_duty_cycle = duty_cycle
        self._shooter_motor.set_control(self._shooter_request.with_output(duty_cycle))

    def set_indexer_speed(self, duty_cycle: float) -> None:
        """Set indexer motor duty cycle (-1.0 to 1.0)."""
        self._indexer_duty_cycle = duty_cycle
        self._indexer_motor.set_control(self._indexer_request.with_output(duty_cycle))

    def stop(self) -> None:
        """Stop both motors."""
        self.set_shooter_speed(0.0)
        self.set_indexer_speed(0.0)
