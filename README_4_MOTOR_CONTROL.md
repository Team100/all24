# Controlling the Motor from the RoboRIO
We finally have the pieces in place to write our first real Control code! Let's open up your Study if you haven't yet, and find the `Robot.java` class. We are going to have to put the motor control code into this class. For now, we will keep it simple and just edit this file directly.

Team100 has code to control various types of motors in the `lib/motor` directory. We are going to use the `NeoCANSparkMotor` which looks like this:

```java
public class NeoCANSparkMotor extends CANSparkMotor {
    /**
     * Note the PID values should be in duty cycle per RPM, i.e. very small. {@link
     * Rev100}
     */
    public NeoCANSparkMotor(
            LoggerFactory parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, new CANSparkMax(canId, MotorType.kBrushless),
                motorPhase, currentLimit, ff, pid);
    }
```

So to use this we need a few things:
1. `int canId`: you should have saved this from a previous README step.
1. `MotorPhase motorPhase`: 
1. `int currentLimit`:
1. `FeedForwar100 ff`:
1. `PIDConstants pid`:

## Initializing the Motor


## Turning the Motor

## Starting/Stopping??

## Attaching to a Button??

