package org.team100.lib.motion.crank;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Example human interface device.
 */
public class HID {
    GenericHID genericHID = new GenericHID(0);

    /** Home the system.  Note this produces motion, it is not a stop command. */
    public void homeWorkstate(Command command) {
        new JoystickButton(genericHID, 1).onTrue(command);
    }

    /** Give the actuators zero input.  This should be "safe torque off." */
    public void stopActuation(Command command) {
        new JoystickButton(genericHID, 10).onTrue(command);
    }

    /** Set follower to feedforward only. */
    public void chooseFF(Command command) {
        new JoystickButton(genericHID, 2).onTrue(command);
    }

    /** Set follower to PID. */
    public void choosePID(Command command) {
        new JoystickButton(genericHID, 3).onTrue(command);
    }

    /** Set the servo to onboard */
    public void onboard(Command command) {
        new JoystickButton(genericHID, 4).onTrue(command);
    }

    /** Set the servo to outboard */
    public void outboard(Command command) {
        new JoystickButton(genericHID, 5).onTrue(command);
    }

    /** Enable the subsystem */
    public void enable(Command command) {
        new JoystickButton(genericHID, 6).onTrue(command);
    }

    /** Disable the subsystem */
    public void disable(Command command) {
        new JoystickButton(genericHID, 7).onTrue(command);
    }

    public void manualConfiguration(Command command) {
        new JoystickButton(genericHID, 12).onTrue(command);
    }

    public void manualActuation(Command command) {
        new JoystickButton(genericHID, 8).onTrue(command);
    }

    /** Run a profile while held. */
    public void runProfile1(Command command) {
        new JoystickButton(genericHID, 4).whileTrue(command);
    }

    /** Run a profile while held. */
    public void runProfile2(Command command) {
        new JoystickButton(genericHID, 5).whileTrue(command);
    }

    /** In reality this would be manual input. */
    public double manual() {
        return 1.0;
    }
}
