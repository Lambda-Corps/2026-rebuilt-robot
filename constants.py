# Target Data: [(DistanceInMeters, DutyCycle)]
TARGET_SHOOTER_DATA = [
    (0.6096, -0.50),
    (1.524, -0.75),
    (2.1336, -0.85)
]

SHOOTER_SPEED_INCREMENT = 0.025
SHOOTER_SPEED_MIN = -0.5
SHOOTER_SPEED_UP = SHOOTER_SPEED_MIN
SHOOTER_SPEED_RIGHT = -0.8
SHOOTER_SPEED_DOWN = -0.7
SHOOTER_SPEED_LEFT = -0.6

MOVE_SPEED_REDUCTION = 0.6  # Added to reduce speed while learning about swerve
ROTATE_SPEED_REDUCTION = 1.0  # NOTE THAT updating _max_speed did not seem to affect speed
DEAD_ZONE = 0.055
EXP_SCALING = 1.3
