package org.team100.lib.motion.example1d.crank;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Example human interface device. */
public class CrankHID {
    GenericHID genericHID = new GenericHID(0);

    /** Stop while held. */
    public void chooseStop(Command command) {
        new JoystickButton(genericHID, 1).onTrue(command);
    }

    /** Set follower to feedforward only. */
    public void chooseFF(Command command) {
        new JoystickButton(genericHID, 2).onTrue(command);
    }

    /** Set follower to PID. */
    public void choosePID(Command command) {
        new JoystickButton(genericHID, 3).onTrue(command);
    }

    /** Run a profile while held. */
    public void runProfile1(Command command) {
        new JoystickButton(genericHID, 4).whileTrue(command);
    }

    /** Run a profile while held. */
    public void runProfile2(Command command) {
        new JoystickButton(genericHID, 4).whileTrue(command);
    }

    /** In reality this would be manual input. */
    public double manual() {
        return 1.0;
    }
}
