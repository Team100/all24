package org.team100.lib.hid;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Represents the HID used by the "operator" role, which typically controls
 * everything other than the drivetrain.
 */
public interface OperatorControl {

    default String getHIDName() {
        return "No HID Found!!";
    }

    default Trigger doSomething() {
        return new Trigger(() -> false);
    }
}
