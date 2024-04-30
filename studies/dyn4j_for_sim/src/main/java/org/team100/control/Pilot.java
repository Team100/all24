package org.team100.control;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

public interface Pilot {
    /**
     * In comp this is controller units [-1,1] but here we want the autopilot to use
     * field units, so the human uses field units too.
     */
    FieldRelativeVelocity driveVelocity();

    boolean intake();

    boolean outtake();

    boolean shoot();

    boolean lob();

    boolean amp();

    boolean rotateToShoot();

    boolean driveToSpeaker();
}
