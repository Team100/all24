package org.team100.lib.timing;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.timing.TimingConstraint.NonNegativeDouble;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.util.Util;

/**
 * Distance is always positive.
 * Velocity is always positive.
 * Max accel is positive.
 * Min accel is negative.
 */
public class TimingUtil {
    private static final double kEpsilon = 1e-6;

    private final List<TimingConstraint> m_constraints;
    private final double m_velocityLimit;
    private final double m_absAccelerationLimit;

    public TimingUtil(
            List<TimingConstraint> constraints,
            double velocityLimit,
            double absAccelerationLimit) {
        m_constraints = constraints;
        m_velocityLimit = velocityLimit;
        m_absAccelerationLimit = absAccelerationLimit;
    }

    /**
     * sample the path evenly by distance, and then assign times to each sample.
     */
    public Trajectory100 timeParameterizeTrajectory(
            PathDistanceSampler sampler,
            double step,
            double start_vel,
            double end_vel) {
        try {
            double maxDistance = sampler.getMaxDistance();
            if (maxDistance == 0)
                throw new IllegalArgumentException();
            int num_states = (int) Math.ceil(maxDistance / step + 1);
            List<Pose2dWithMotion> samples = new ArrayList<>(num_states);
            for (int i = 0; i < num_states; ++i) {
                samples.add(sampler.sample(Math.min(i * step, maxDistance)).state());
            }
            return timeParameterizeTrajectory(samples, start_vel, end_vel);
        } catch (TimingException e) {
            e.printStackTrace();
            Util.warn("Timing exception");
            return new Trajectory100();
        }
    }

    /**
     * input is some set of samples (could be evenly sampled or not), output is
     * these same samples with time.
     */
    private Trajectory100 timeParameterizeTrajectory(
            List<Pose2dWithMotion> samples,
            double start_vel,
            double end_vel) throws TimingException {
        List<ConstrainedState> constrainedStates = forwardPass(samples, start_vel);
        Pose2dWithMotion lastState = samples.get(samples.size() - 1);
        backwardsPass(lastState, end_vel, constrainedStates);
        return integrate(constrainedStates);
    }

    /**
     * Forward pass.
     * 
     * We look at pairs of consecutive states, where the start state has already
     * been velocity parameterized (though we may adjust the velocity downwards
     * during the backwards pass). We wish to find an acceleration that is
     * admissible at both the start and end state, as well as an admissible end
     * velocity. If there is no admissible end velocity or acceleration, we set the
     * end velocity to the state's maximum allowed velocity and will repair the
     * acceleration during the backward pass (by slowing down the predecessor).
     */
    private List<ConstrainedState> forwardPass(List<Pose2dWithMotion> samples, double start_vel) {
        ConstrainedState predecessor = new ConstrainedState(samples.get(0), 0);
        predecessor.vel = start_vel;
        predecessor.min_acceleration = -m_absAccelerationLimit;
        predecessor.max_acceleration = m_absAccelerationLimit;

        // work forward through the samples
        List<ConstrainedState> constrainedStates = new ArrayList<>(samples.size());
        for (Pose2dWithMotion sample : samples) {
            double ds = sample.distance(predecessor.state);
            ConstrainedState constrainedState = new ConstrainedState(sample, ds + predecessor.distance);
            constrainedStates.add(constrainedState);
            forwardWork(predecessor, constrainedState);
            predecessor = constrainedState;
        }
        return constrainedStates;
    }

    private void forwardWork(ConstrainedState s0, ConstrainedState s1) {
        // constant-twist path length between states
        // note this is zero for turn-in-place.
        double ds = s1.state.distance(s0.state);

        // We may need to iterate to find the maximum end velocity and common
        // acceleration, since acceleration limits may be a function of velocity.
        while (true) {
            // first try the previous state accel to get the new state velocity
            double v1 = v1(s0.vel, s0.max_acceleration, ds);
            s1.vel = Math.min(m_velocityLimit, v1);

            // also use max accels for the new state accels
            s1.min_acceleration = -m_absAccelerationLimit;
            s1.max_acceleration = m_absAccelerationLimit;

            // reduce velocity according to constraints
            s1.clampVelocity(m_constraints);

            // reduce accel according to constraints
            s1.clampAccel(m_constraints);

            // motionless
            if (Math.abs(ds) < kEpsilon) {
                return;
            }

            double accel = accel(s0.vel, s1.vel, ds);
            if (accel > s1.max_acceleration + kEpsilon) {
                // implied accel is too high because v1 is too high, perhaps because
                // a0 was too high, try again with the (lower) constrained value
                s0.max_acceleration = s1.max_acceleration;
                continue;
            }
            if (accel > s0.min_acceleration + kEpsilon) {
                // set the previous state accel to whatever the constrained velocity implies
                s0.max_acceleration = accel;
            }
            return;
        }
    }

    /**
     * Backwards pass
     */
    private void backwardsPass(
            Pose2dWithMotion lastState,
            double end_velocity,
            List<ConstrainedState> constrainedStates) {
        // "successor" comes before in the backwards walk. start with the last state.
        ConstrainedState endState = constrainedStates.get(constrainedStates.size() - 1);
        ConstrainedState successor = new ConstrainedState(lastState, endState.distance);
        successor.vel = end_velocity;
        successor.min_acceleration = -m_absAccelerationLimit;
        successor.max_acceleration = m_absAccelerationLimit;

        // work backwards through the states list
        for (int i = constrainedStates.size() - 1; i >= 0; --i) {
            ConstrainedState constrainedState = constrainedStates.get(i);
            backwardsWork(constrainedState, successor);
            successor = constrainedState;
        }
    }

    /** s0 is earlier, s1 is "successor", we're walking backwards. */
    private void backwardsWork(ConstrainedState s0, ConstrainedState s1) {
        // backwards (negative) distance from successor to initial state.
        double ds = s0.distance - s1.distance;
        if (ds > 0) {
            // must be negative if we're walking backwards.
            throw new IllegalStateException();
        }

        while (true) {
            // s0 velocity can't be more than the accel implies
            // so this is actually an estimate for v0
            // min a is negative, ds is negative, so v0 is faster than v1
            double v0 = v1(s1.vel, s1.min_acceleration, ds);

            if (s0.vel <= v0) {
                // s0 v is slower than implied v0, which means
                // that actual accel is larger than the min, so we're fine
                // No new limits to impose.
                return;
            }
            // s0 v is too fast, turn it down to obey v1 min accel.
            s0.vel = v0;

            s0.clampAccel(m_constraints);

            // motionless
            if (Math.abs(ds) < kEpsilon) {
                return;
            }

            // implied accel using the constrained v0
            double accel = accel(s1.vel, s0.vel, ds);
            if (accel < s0.min_acceleration - kEpsilon) {
                // accel is too low which implies that s1 accel is too low, try again
                s1.min_acceleration = s0.min_acceleration;
                continue;
            }
            // set final accel to the implied value
            s1.min_acceleration = accel;
            return;
        }
    }

    /**
     * Integrate the constrained states forward in time to obtain the TimedStates.
     * 
     * last state accel is always zero, which might be wrong.
     */
    private static Trajectory100 integrate(List<ConstrainedState> states) throws TimingException {
        List<TimedPose> poses = new ArrayList<>(states.size());
        double time = 0.0; // time along path
        double distance = 0.0; // distance along path
        double v0 = 0.0;
        for (int i = 0; i < states.size(); ++i) {
            ConstrainedState state = states.get(i);
            double ds = state.distance - distance;
            double v1 = state.vel;
            double dt = 0.0;
            if (i > 0) {
                double prevAccel = accel(v0, v1, ds);
                poses.get(i - 1).set_acceleration(prevAccel);
                dt = dt(v0, v1, ds, prevAccel);
            }
            time += dt;
            if (Double.isNaN(time) || Double.isInfinite(time)) {
                throw new TimingException();
            }
            poses.add(new TimedPose(state.state, time, v1, 0));
            v0 = v1;
            distance = state.distance;
        }
        return new Trajectory100(poses);
    }

    private static double dt(
            double v0,
            double v1,
            double ds,
            double accel) throws TimingException {
        if (Math.abs(accel) > kEpsilon) {
            return (v1 - v0) / accel;
        }
        if (Math.abs(v0) > kEpsilon) {
            return ds / v0;
        }
        throw new TimingException();
    }

    /**
     * Return final velocity, v1, given initial velocity, v0, and acceleration over
     * distance ds.
     * 
     * v1 = sqrt(v0^2 + 2ads)
     * 
     * note a can be negative.
     * 
     * note ds can be negative, which implies backwards time
     */
    static double v1(double v0, double a, double ds) {
        /*
         * a = dv/dt
         * v = ds/dt
         * dt = ds/v
         * a = v dv/ds
         * a = v (v1-v0)/ds
         * v = (v0+v1)/2
         * a = (v0+v1)(v1-v0)/2ds
         * a = (v1^2 - v0^2)/2ds
         * 2*a*ds = v1^2 - v0^2
         * v1 = sqrt(v0^2 + 2*a*ds)
         */
        return Math.sqrt(v0 * v0 + 2.0 * a * ds);
    }

    /**
     * Return acceleration implied by the change in velocity (v0 to v1)
     * over the distance, ds.
     * 
     * a = (v1^2 - v0^2) / 2ds
     * 
     * note ds can be negative, which implies negative time.
     */
    static double accel(double v0, double v1, double ds) {
        /*
         * a = dv/dt
         * v = ds/dt
         * dt = ds/v
         * a = v dv/ds
         * a = v (v1-v0)/ds
         * v = (v0+v1)/2
         * a = (v0+v1)(v1-v0)/2ds
         * a = (v1^2 - v0^2)/2ds
         */
        return (v1 * v1 - v0 * v0) / (2.0 * ds);
    }

    static class ConstrainedState {
        public final Pose2dWithMotion state;
        /** Cumulative distance along the path */
        public final double distance;
        public double vel;
        public double min_acceleration;
        public double max_acceleration;

        public ConstrainedState(Pose2dWithMotion state, double distance) {
            this.state = state;
            this.distance = distance;
        }

        /**
         * Clamp state velocity to constraints.
         */
        public void clampVelocity(List<TimingConstraint> constraints) {
            for (TimingConstraint constraint : constraints) {
                NonNegativeDouble constraintVel = constraint.getMaxVelocity(state);
                vel = Math.min(vel, constraintVel.getValue());
            }
        }

        /**
         * Clamp constraint state accelerations to the constraints.
         */
        public void clampAccel(List<TimingConstraint> constraints) {
            for (TimingConstraint constraint : constraints) {
                TimingConstraint.MinMaxAcceleration min_max_accel = constraint
                        .getMinMaxAcceleration(state, vel);
                min_acceleration = Math.max(
                        min_acceleration,
                        min_max_accel.getMinAccel());
                max_acceleration = Math.min(
                        max_acceleration,
                        min_max_accel.getMaxAccel());
            }

        }

        @Override
        public String toString() {
            return state.toString() + ", distance: " + distance + ", vel: " + vel + ", " +
                    "min_acceleration: " + min_acceleration + ", max_acceleration: " + max_acceleration;
        }
    }

    public static class TimingException extends Exception {
    }
}