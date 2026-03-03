"""Physics simulation module for RobotPy sim.

This module provides physics simulation support for the robot when running
in simulation mode. It interfaces with WPILib's simulation framework to:
- Update drivetrain physics based on motor inputs
- Provide robot pose updates to VisionSubsystem simulation
- Handle battery voltage simulation
- Provide simulated joystick input for testing

The PhysicsEngine class is automatically discovered and used by RobotPy sim.
"""

from wpilib import RobotBase, RobotController
from wpilib.simulation import DriverStationSim, RoboRioSim, XboxControllerSim
from pyfrc.physics.core import PhysicsInterface

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot import Robot


class PhysicsEngine:
    """
    Physics simulation engine for RobotPy sim.

    This class is automatically instantiated by RobotPy when running
    'robotpy sim'. It updates physics models and simulation state.
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        """
        Initialize the physics engine.

        Args:
            physics_controller: Interface to WPILib simulation framework
            robot: The Robot instance being simulated
        """
        self.physics_controller = physics_controller
        self.robot = robot

        # Store references to subsystems for simulation updates
        # These will be populated after RobotContainer is created
        self.drivetrain = None
        self.vision_subsystem = None

        # Set up simulated Xbox controller for joystick input
        # This allows controlling the robot in simulation without physical hardware
        # The XboxControllerSim is automatically used by CommandXboxController in simulation
        self._controller_sim = XboxControllerSim(0)

        # Also try to set up a direct HID reference for reading input
        from wpilib.simulation import GenericHIDSim

        self._hid_sim = GenericHIDSim(0)

        # Try to get subsystems from RobotContainer
        if hasattr(robot, "container"):
            self.drivetrain = getattr(robot.container, "drivetrain", None)
            self.vision_subsystem = getattr(robot.container, "_vision_subsystem", None)
            print(f"[Physics] Found drivetrain: {self.drivetrain is not None}")
            print(
                f"[Physics] Found vision_subsystem: {self.vision_subsystem is not None}"
            )
        else:
            print(
                "[Physics] Robot container not yet created (will be discovered later)"
            )

        # Simulation state
        self._last_time = 0.0

        print("[Physics] Simulation engine initialized")

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called periodically during simulation to update physics.

        This method is called automatically by the RobotPy simulation framework
        at approximately 50Hz (every 20ms).

        Args:
            now: Current simulation time in seconds
            tm_diff: Time delta since last update (should be ~0.02s)
        """
        # DO NOT force enable/disable - let the Sim GUI control the robot state
        # DriverStationSim.setEnabled(True)  # REMOVED: This was overriding the GUI
        # DriverStationSim.setAutonomous(False)  # REMOVED: This was overriding the GUI

        # Discover subsystems if not yet found (RobotContainer may have been created since init)
        if self.drivetrain is None or self.vision_subsystem is None:
            self._discover_subsystems()

        # Update drivetrain simulation if available
        if self.drivetrain is not None:
            self._update_drivetrain_sim(tm_diff)

        # Update vision simulation
        if self.vision_subsystem is not None:
            try:
                self.vision_subsystem.simulationPeriodic()
            except Exception as e:
                # Log error but don't crash simulation
                print(f"[Physics] Vision simulation error: {e}")
                import traceback

                traceback.print_exc()

        # Simulate battery voltage based on current draw
        # This is a simplified model - full implementation would track motor currents
        battery_voltage = RobotController.getBatteryVoltage()
        if battery_voltage < 11.0:
            # Simulate battery sag under load (simplified)
            RoboRioSim.setVInVoltage(12.0)

        self._last_time = now

    def _update_drivetrain_sim(self, tm_diff: float) -> None:
        """
        Update drivetrain physics simulation.

        Args:
            tm_diff: Time delta since last update
        """
        # Check if drivetrain has simulation support
        # CTRE swerve drivetrains handle their own physics internally
        # but we need to ensure the pose is updated for vision simulation

        if hasattr(self.drivetrain, "update_simulation"):
            # Some drivetrain implementations provide explicit sim update
            self.drivetrain.update_simulation(tm_diff)
        elif hasattr(self.drivetrain, "simulationPeriodic"):
            # Standard WPILib pattern
            self.drivetrain.simulationPeriodic()

        # The drivetrain's pose is automatically tracked by the swerve modules
        # VisionSubsystem will query it via get_state().pose or get_sim_pose()

    def _discover_subsystems(self):
        """Discover robot subsystems if not yet found."""
        if not hasattr(self.robot, "container"):
            return

        if self.drivetrain is None:
            self.drivetrain = getattr(self.robot.container, "drivetrain", None)
            if self.drivetrain is not None:
                print("[Physics] Discovered drivetrain")

        if self.vision_subsystem is None:
            self.vision_subsystem = getattr(
                self.robot.container, "_vision_subsystem", None
            )
            if self.vision_subsystem is not None:
                print("[Physics] Discovered vision_subsystem")

    def initialize(self, hardware: "Hardware") -> None:
        """
        Called once at simulation startup to initialize hardware interfaces.

        Args:
            hardware: Hardware abstraction layer
        """
        pass
