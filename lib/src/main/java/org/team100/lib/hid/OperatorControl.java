package org.team100.lib.hid;

import org.team100.lib.dashboard.Glassy;

/**
 * Represents the HID used by the "operator" role, which typically controls
 * everything other than the drivetrain.
 */
public interface OperatorControl extends Glassy {

    default String getHIDName() {
        return "No HID Found!!";
    }

    default boolean pivotToAmpPosition() {
        return false;
    }

    default boolean outtake() {
        return false;
    }

    default boolean feed() {
        return false;
    }

    default boolean intake() {
        return false;
    }

    default boolean ramp() {
        return false;
    }

    // this exists to bind to commands we don't want to run,
    // but we don't want them to rot either.
    default boolean never() {
        return false;
    }

    default boolean selfTestEnable() {
        return false;
    }

    default double leftClimb() {
        return 0;
    }

    default double rightClimb() {
        return 0;
    }

    /** Put the climber hooks in the "up" position. */
    default boolean climbUpPosition() {
        return false;
    }

    /** Put the climber hooks in the "down" position. */
    default boolean climbDownPosition() {
        return false;
    }

    default boolean feedToAmp() {
        return false;
    }

    default boolean outtakeFromAmp() {
        return false;
    }

    default boolean testShoot() {
        return false;
    }

    @Override
    default String getGlassName() {
        return "DriverControl";
    }
}
