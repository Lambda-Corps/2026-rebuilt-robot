"""
Centralized logging utility for controlling debug output verbosity.

LOG_VERBOSITY levels:
  0 = Off (no debug output)
  1 = Infrequent events only (mode changes, enable/disable)
  5 = Moderate frequency (speed changes)
  10 = All output including high-frequency periodic calls
"""

from wpilib import SmartDashboard

LOG_VERBOSITY = 0

_cycle_counter = 0


def log_debug(message: str, min_verbosity: int = 1) -> None:
    if LOG_VERBOSITY >= min_verbosity:
        print(message)


def log_smartdashboard_number(
    key: str, value: float, min_verbosity: int = 1, update_frequency: int = 10
) -> None:
    global _cycle_counter
    if LOG_VERBOSITY >= min_verbosity:
        _cycle_counter += 1
        if _cycle_counter % update_frequency == 0:
            SmartDashboard.putNumber(key, value)


def log_smartdashboard_boolean(
    key: str, value: bool, min_verbosity: int = 1, update_frequency: int = 10
) -> None:
    global _cycle_counter
    if LOG_VERBOSITY >= min_verbosity:
        _cycle_counter += 1
        if _cycle_counter % update_frequency == 0:
            SmartDashboard.putBoolean(key, value)


def log_smartdashboard_string(
    key: str, value: str, min_verbosity: int = 1, update_frequency: int = 10
) -> None:
    global _cycle_counter
    if LOG_VERBOSITY >= min_verbosity:
        _cycle_counter += 1
        if _cycle_counter % update_frequency == 0:
            SmartDashboard.putString(key, value)
