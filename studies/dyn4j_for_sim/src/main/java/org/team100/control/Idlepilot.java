package org.team100.control;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

/**
 * Pilot that does nothing.
 */
public class Idlepilot implements Pilot {

    @Override
    public FieldRelativeVelocity driveVelocity() {
        return new FieldRelativeVelocity(0, 0, 0);
    }

    @Override
    public boolean intake() {
        return false;
    }

    @Override
    public boolean outtake() {
        return false;
    }

    @Override
    public boolean shoot() {
        return false;
    }

    @Override
    public boolean lob() {
        return false;
    }

    @Override
    public boolean amp() {
        return false;
    }

    @Override
    public boolean rotateToShoot() {
        return false;
    }

    @Override
    public  boolean driveToSpeaker() {
        return false;
    }

}
