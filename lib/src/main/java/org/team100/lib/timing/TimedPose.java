package org.team100.lib.timing;

import edu.wpi.first.math.MathUtil;

import java.text.DecimalFormat;

import org.team100.lib.geometry.Pose2dWithMotion;

/** Timed Pose now includes the heading part, timed rotation is gone. */
public class TimedPose {
    private final Pose2dWithMotion state_;
    protected double timeS; // Time we achieve this state.
    protected double velocityM_S; // ds/dt
    protected double accelM_S_S; // d^2s/dt^2

    public TimedPose(final Pose2dWithMotion state) {
        this(state, 0, 0, 0);
    }

    public TimedPose(final Pose2dWithMotion state, double t, double velocity, double acceleration) {
        state_ = state;
        timeS = t;
        velocityM_S = velocity;
        accelM_S_S = acceleration;
    }

    public Pose2dWithMotion state() {
        return state_;
    }

    public void setTimeS(double t) {
        timeS = t;
    }

    public double getTimeS() {
        return timeS;
    }

    public void set_velocity(double velocity) {
        velocityM_S = velocity;
    }

    public double velocityM_S() {
        return velocityM_S;
    }

    public void set_acceleration(double acceleration) {
        accelM_S_S = acceleration;
    }

    /** this means acceleration along the path, not centripetal acceleration. */
    public double acceleration() {
        return accelM_S_S;
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return state().getPose().toString()
                + ", curve: " + fmt.format(state().getCurvature())
                + ", time: " + fmt.format(getTimeS())
                + ", vel: " + fmt.format(velocityM_S())
                + ", acc: " + fmt.format(acceleration());
    }

    public TimedPose interpolate2(TimedPose other, double x) {
        final double new_t = MathUtil.interpolate(getTimeS(), other.getTimeS(), x);
        final double delta_t = new_t - getTimeS();
        if (delta_t < 0.0) {
            return other.interpolate2(this, 1.0 - x);
        }
        boolean reversing = velocityM_S() < 0.0 || (Math.abs(velocityM_S() - 0.0) <= 1e-12 && acceleration() < 0.0);
        final double new_v = velocityM_S() + acceleration() * delta_t;
        final double new_s = (reversing ? -1.0 : 1.0)
                * (velocityM_S() * delta_t + .5 * acceleration() * delta_t * delta_t);

        double interpolant = new_s / state().distance(other.state());
        if (Double.isNaN(interpolant)) {
            interpolant = 1.0;
        }

        return new TimedPose(
                state().interpolate(other.state(), interpolant),
                new_t,
                new_v,
                acceleration());
    }

    public double distance(TimedPose other) {
        return state().distance(other.state());
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof TimedPose)) {
            System.out.println("wrong type");
            return false;
        }
        TimedPose ts = (TimedPose) other;
        boolean stateEqual = state().equals(ts.state());
        if (!stateEqual) {
            System.out.println("state not equal");
            return false;
        }
        boolean timeEqual = Math.abs(getTimeS() - ts.getTimeS()) <= 1e-12;
        if (!timeEqual) {
            System.out.println("time not equal");
            return false;
        }
        return true;
    }
}
