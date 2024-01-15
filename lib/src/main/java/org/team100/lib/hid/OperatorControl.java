package org.team100.lib.hid;

import edu.wpi.first.wpilibj2.command.Command;
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

    default void index(Command command) {}

    default void shooter(Command command) {}

    default void intake(Command command) {}

    default double climberState() {
        return 0;
    }


    default double lower() {
        return 0;
    }

    default double upper() {
        return 0;
    }

    default double elevator() {
        return 0;
    }

    // this exists to bind to commands we don't want to run,
    // but we don't want them to rot either.
    default Trigger never() {
        return new Trigger(() -> false);
    }
}
