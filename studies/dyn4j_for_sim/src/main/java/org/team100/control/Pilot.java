package org.team100.control;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

public interface Pilot {
    FieldRelativeVelocity driveVelocity();

    boolean intake();

    boolean outtake();

    boolean shoot();

    boolean lob();

    boolean amp();

    boolean rotateToShoot();

    boolean driveToSpeaker();
}
