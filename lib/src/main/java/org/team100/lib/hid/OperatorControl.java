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

    default Trigger index() {
        return new Trigger(() -> false);
    }

    default Trigger shooter() {
        return new Trigger(() -> false);
    }

    default double shooterSpeed() {
        return 0;
    }

    default Trigger outtake() {
        return new Trigger(() -> false);
    }

    default Trigger intake() {
        return new Trigger(() -> false);
    }

    default boolean indexState() {
        return false;
    }

    default double ampState(){
        return 0;
    }

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

    default boolean selfTestEnable() {
        return false;
    }
}
