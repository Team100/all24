package org.team100.lib.timing;

import java.util.List;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.timing.TimingConstraint.NonNegativeDouble;

class ConstrainedState {
    // using MAX_VALUE tickles some bugs
    private static final double maxV = 100;
    private static final double maxA = 100;
    private final Pose2dWithMotion state;
    /** Cumulative distance along the path */
    private final double distance;
    private double vel;
    private double min_acceleration;
    private double max_acceleration;

    public ConstrainedState(Pose2dWithMotion state, double distance) {
        this.state = state;
        this.distance = distance;
        setVel(maxV);
        setMin_acceleration(-maxA);
        setMax_acceleration(maxA);
    }

    /**
     * Clamp state velocity to constraints.
     */
    public void clampVelocity(List<TimingConstraint> constraints) {
        for (TimingConstraint constraint : constraints) {
            NonNegativeDouble constraintVel = constraint.getMaxVelocity(state);
            double value = constraintVel.getValue();
            setVel(Math.min(getVel(), value));
        }
    }

    /**
     * Clamp constraint state accelerations to the constraints.
     */
    public void clampAccel(List<TimingConstraint> constraints) {
        for (TimingConstraint constraint : constraints) {
            TimingConstraint.MinMaxAcceleration min_max_accel = constraint
                    .getMinMaxAcceleration(state, getVel());
            double minAccel = min_max_accel.getMinAccel();
            if (Double.isNaN(minAccel))
                throw new IllegalArgumentException();
            min_acceleration = Math.max(
                    min_acceleration,
                    minAccel);
            double maxAccel = min_max_accel.getMaxAccel();
            if (Double.isNaN(maxAccel))
                throw new IllegalArgumentException();
            max_acceleration = Math.min(
                    max_acceleration,
                    maxAccel);
        }

    }

    public Pose2dWithMotion getState() {
        return state;
    }

    public double getDistance() {
        return distance;
    }

    public double getVel() {
        return vel;
    }

    public void setVel(double vel) {
        if (Double.isNaN(vel))
            throw new IllegalArgumentException();
        this.vel = vel;
    }

    public double getMin_acceleration() {
        return min_acceleration;
    }

    public void setMin_acceleration(double min_acceleration) {
        this.min_acceleration = min_acceleration;
    }

    public double getMax_acceleration() {
        return max_acceleration;
    }

    public void setMax_acceleration(double max_acceleration) {
        this.max_acceleration = max_acceleration;
    }

    @Override
    public String toString() {
        return state.toString() + ", distance: " + distance + ", vel: " + getVel() + ", " +
                "min_acceleration: " + min_acceleration + ", max_acceleration: " + max_acceleration;
    }
}