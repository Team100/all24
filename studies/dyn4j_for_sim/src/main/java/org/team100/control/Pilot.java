package org.team100.control;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

public interface Pilot {
    /**
     * In comp this is controller units [-1,1] but here we want the autopilot to use
     * field units, so the human uses field units too.
     */
    default FieldRelativeVelocity driveVelocity() {
        return new FieldRelativeVelocity(0, 0, 0);
    }

    default boolean intake() {
        return false;
    }

    default boolean outtake() {
        return false;
    }

    default boolean shoot() {
        return false;
    }

    default boolean lob() {
        return false;
    }

    default boolean amp() {
        return false;
    }

    default boolean rotateToShoot() {
        return false;
    }

    default boolean driveToSpeaker() {
        return false;
    }

    default boolean driveToAmp() {
        return false;
    }

    default boolean driveToSource() {
        return false;
    }

    default boolean driveToPass() {
        return false;
    }

    default boolean shootCommand() {
        return false;
    }

    void onEnd();

    /**
     * The controller states here are monitored with triggers, which notice
     * **EDGES** so we need to start with everything off, and wait until the trigger
     * is litening.
     */
    void begin();

    /** Turn off all the outputs. */
    void reset();

    default void periodic() {
        
    }
}
