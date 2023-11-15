package frc.robot.consoles;

/** Corresponds to arduino knobs.ino.
 * 
 * Use getRawAxis and getRawButton to access the encoders.
 */
public class KnobConsole extends BaseConsole {
    public KnobConsole() {
        super(portFromName("Knobs"));
    }
}
