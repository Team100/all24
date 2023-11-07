package org.team100.lib.motion.example1d;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Example human interface device. */
public class HID {
    GenericHID genericHID = new GenericHID(0);

    /** Stop while held. */
    public void chooseStop(Command command) {
        new JoystickButton(genericHID, 1).onTrue(command);
    }
    
    /** Run FF while held. */
    public void chooseFF(Command command) {
        new JoystickButton(genericHID, 2).whileTrue(command);
    }

    /** Run PID while held. */
    public void choosePID(Command command) {
        new JoystickButton(genericHID, 3).whileTrue(command);
    }

    /** In reality this would be manual input. */
    public double manual() {
        return 1.0;
    }

}
