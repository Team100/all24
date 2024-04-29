package org.team100.control;

/**
 * Autopilot implements strategy.
 */
public class Autopilot implements Pilot {

    @Override
    public double getLeftX() {
        return 0;
    }

    @Override
    public double getRightY() {
        return 0;
    }

    @Override
    public double getRightX() {
        return 0;
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

}
