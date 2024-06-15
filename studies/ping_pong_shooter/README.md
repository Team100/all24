# Ping Pong Ball Shooter

This project is intended to illustrate the software patterns used by Team 100:

- Subsystem.periodic() is used for measurement.
- Command.execute() drives actuation, to minimize delay.
- Full state feedback is preferred.
- Classes are small.
- Control binding in RobotContainer, using command factories in subsystems.
- Comments explain where things came from, include links.
- Members are private final where possible.
- Network tables output is used everywhere.

The ping pong ball shooter includes three subsystems:

- flywheels
- feeder
- elevation angle

Hardware used in this project includes:

- Feeder servo uses continuous rotation and positional feedback, see <https://www.pololu.com/product/3432>
- Elevation servo uses positional servo with positional feedback, see <https://www.pololu.com/product/3442>
- Flywheels use Pololu motors with quadrature encoders, see <https://www.pololu.com/product/4789> and <https://www.pololu.com/product/4761>
- Pololu dual motor controller for flywheels, see <https://www.pololu.com/product/2137>

Note that none of the code here implements the WPILIb "Motor Safety" concept.
Instead, the servos are enabled and disabled in teleopInit() and teleopExit().