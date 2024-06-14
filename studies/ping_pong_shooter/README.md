# Ping Pong Ball Shooter

This project is intended to illustrate the software patterns used by Team 100:

- Action is driven by command.execute(); subsystem.periodic() is unused (because execute() is immediate, whereas periodic() is delayed by one period)
- Full state control is preferred.

The ping pong ball shooter includes three subsystems:

- flywheels
- feeder
- elevation angle

Hardware used in this project includes:

- Feeder servo uses continuous rotation and positional feedback, see <https://www.pololu.com/product/3432>
- Elevation servo uses positional servo with positional feedback, see <https://www.pololu.com/product/3442>
- Flywheels use Pololu motors with quadrature encoders, see <https://www.pololu.com/product/4789> and <https://www.pololu.com/product/4761>
- Pololu dual motor controller for flywheels, see <https://www.pololu.com/product/2137>
