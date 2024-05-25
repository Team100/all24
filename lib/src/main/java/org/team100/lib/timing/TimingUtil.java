package org.team100.lib.timing;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.path.PathIndexSampler;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.util.Util;

public class TimingUtil {
    private static final double kEpsilon = 1e-6;

    public static Trajectory100 timeParameterizeTrajectory(
            final PathDistanceSampler distance_view,
            double step_size,
            final List<TimingConstraint> constraints,
            double start_velocity,
            double end_velocity,
            double max_velocity,
            double max_abs_acceleration) {
        try {
            final int num_states = (int) Math.ceil(distance_view.getMaxDistance() / step_size + 1);
            List<Pose2dWithMotion> states = new ArrayList<>(num_states);
            for (int i = 0; i < num_states; ++i) {
                states.add(distance_view.sample(Math.min(i * step_size, distance_view.getMaxDistance())).state());
            }
            return timeParameterizeTrajectory(states, constraints, start_velocity, end_velocity,
                    max_velocity, max_abs_acceleration);
        } catch (TimingException e) {
            Util.warn("Timing exception");
            return new Trajectory100();
        }
    }

    public static Trajectory100 timeParameterizeTrajectory(
        final PathIndexSampler distance_view,
        double step_size,
        final List<TimingConstraint> constraints,
        double start_velocity,
        double end_velocity,
        double max_velocity,
        double max_abs_acceleration) {
    System.out.println("ASDF+=======================");
    try {
        final int num_states = (int) Math.ceil(distance_view.getMaxIndex() / step_size + 1);
        System.out.println("states " + num_states);
        List<Pose2dWithMotion> states = new ArrayList<>(num_states);
        for (int i = 0; i < num_states; ++i) {
            states.add(distance_view.sample(Math.min(i * step_size, distance_view.getMaxIndex())).state());
        }
        return timeParameterizeTrajectory(states, constraints, start_velocity, end_velocity,
                max_velocity, max_abs_acceleration);
    } catch (TimingException e) {
        Util.warn("Timing exception");
        return new Trajectory100();
    }
}

    private static Trajectory100 timeParameterizeTrajectory(
            final List<Pose2dWithMotion> states,
            final List<TimingConstraint> constraints,
            double start_velocity,
            double end_velocity,
            double max_velocity,
            double max_abs_acceleration) throws TimingException {

        List<ConstrainedState> constraint_states = forwardPass(
                states,
                constraints,
                start_velocity,
                max_velocity,
                max_abs_acceleration);

        backwardsPass(
                states,
                constraints,
                end_velocity,
                max_abs_acceleration,
                constraint_states);

        return integrate(states, constraint_states);
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
    private static List<ConstrainedState> forwardPass(
            final List<Pose2dWithMotion> states,
            final List<TimingConstraint> constraints,
            double start_velocity,
            double max_velocity,
            double max_abs_acceleration) throws TimingException {

        List<ConstrainedState> constraint_states = new ArrayList<>(states.size());

        ConstrainedState predecessor = new ConstrainedState();
        predecessor.state = states.get(0);
        predecessor.distance = 0.0;
        predecessor.max_velocity = start_velocity;
        predecessor.min_acceleration = -max_abs_acceleration;
        predecessor.max_acceleration = max_abs_acceleration;

        // work forward through the state list
        for (Pose2dWithMotion p : states) {
            ConstrainedState constraint_state = new ConstrainedState();
            constraint_states.add(constraint_state);
            constraint_state.state = p;
            forwardWork(constraints, max_velocity, max_abs_acceleration, predecessor, constraint_state);
            predecessor = constraint_state;
        }
        return constraint_states;
    }

    private static void forwardWork(
            final List<TimingConstraint> constraints,
            double max_velocity,
            double max_abs_acceleration,
            ConstrainedState predecessor,
            ConstrainedState constraint_state) throws TimingException {

        // constant-twist path length between states
        final double ds = constraint_state.state.distance(predecessor.state);

        // total path distance so far
        constraint_state.distance = ds + predecessor.distance;

        // We may need to iterate to find the maximum end velocity and common
        // acceleration, since acceleration limits may be a function of velocity.
        while (true) {
            // Enforce global max velocity and max reachable velocity by global acceleration
            // limit.
            // vf = sqrt(vi^2 + 2*a*d)
            constraint_state.max_velocity = Math.min(max_velocity,
                    Math.sqrt(predecessor.max_velocity * predecessor.max_velocity
                            + 2.0 * predecessor.max_acceleration * ds));
            if (Double.isNaN(constraint_state.max_velocity)) {
                throw new TimingException();
            }
            // Enforce global max absolute acceleration.
            constraint_state.min_acceleration = -max_abs_acceleration;
            constraint_state.max_acceleration = max_abs_acceleration;

            // At this point, the state is full constructed, but no constraints have been
            // applied aside from predecessor state max accel.

            clampVelocity(constraints, constraint_state);

            clampAccel(constraints, constraint_state);

            if (ds < kEpsilon) {
                return;
            }
            // If the max acceleration for this constraint state is more conservative than
            // what we had applied, we need to reduce the max accel at the predecessor state
            // and try again. Doing a search would be better.
            final double actual_acceleration = getActual_acceleration(predecessor, constraint_state, ds);
            if (constraint_state.max_acceleration < actual_acceleration - kEpsilon) {
                predecessor.max_acceleration = constraint_state.max_acceleration;
            } else {
                if (actual_acceleration > predecessor.min_acceleration + kEpsilon) {
                    predecessor.max_acceleration = actual_acceleration;
                }
                // If actual acceleration is less than predecessor min accel, we will repair
                // during the backward pass.
                return;
            }
        }
    }

    /**
     * Clamp state velocity to constraints.
     */
    private static void clampVelocity(
            final List<TimingConstraint> constraints,
            ConstrainedState constraint_state) throws TimingException {
        for (final TimingConstraint constraint : constraints) {
            constraint_state.max_velocity = Math.min(constraint_state.max_velocity,
                    constraint.getMaxVelocity(constraint_state.state));
        }
        if (constraint_state.max_velocity < 0.0) {
            // This should never happen if constraints are well-behaved.
            throw new TimingException();
        }
    }

    /**
     * Backwards pass
     */
    private static void backwardsPass(
            final List<Pose2dWithMotion> states,
            final List<TimingConstraint> constraints,
            double end_velocity,
            double max_abs_acceleration,
            List<ConstrainedState> constraint_states) throws TimingException {
        // "successor" comes before in the backwards walk. start with the last state.
        ConstrainedState successor = new ConstrainedState();
        successor.state = states.get(states.size() - 1);
        successor.distance = constraint_states.get(states.size() - 1).distance;
        successor.max_velocity = end_velocity;
        successor.min_acceleration = -max_abs_acceleration;
        successor.max_acceleration = max_abs_acceleration;

        // work backwards through the states list
        for (int i = states.size() - 1; i >= 0; --i) {
            ConstrainedState constraint_state = constraint_states.get(i);
            backwardsWork(constraints, successor, constraint_state);
            successor = constraint_state;
        }
    }

    private static void backwardsWork(
            final List<TimingConstraint> constraints,
            ConstrainedState successor,
            ConstrainedState constraint_state) throws TimingException {
        // distance from this state to the successor, as a negative number.
        final double ds = constraint_state.distance - successor.distance;

        while (true) {
            // Enforce reverse max reachable velocity limit.
            // vf = sqrt(vi^2 + 2*a*d), where vi = successor.
            // entrance velocity can't be more than the accel implies
            final double new_max_velocity = Math.sqrt(successor.max_velocity * successor.max_velocity
                    + 2.0 * successor.min_acceleration * ds);
            if (new_max_velocity >= constraint_state.max_velocity) {
                // No new limits to impose.
                return;
            }
            constraint_state.max_velocity = new_max_velocity;
            if (Double.isNaN(constraint_state.max_velocity)) {
                throw new TimingException();
            }

            clampAccel(constraints, constraint_state);

            if (ds > kEpsilon) {
                return;
            }
            // If the min acceleration for this constraint state is more conservative than
            // what we have applied, we need to reduce the min accel and try again. Doing a
            // search would be better.
            // accel based on state velocities:
            final double actual_acceleration = getActual_acceleration(successor, constraint_state, ds);

            if (constraint_state.min_acceleration > actual_acceleration + kEpsilon) {
                // state accel is too low
                successor.min_acceleration = constraint_state.min_acceleration;
            } else {
                successor.min_acceleration = actual_acceleration;
                return;
            }
        }
    }

    /**
     * This calculates acceleration based on the change in velocity and the change
     * in distance, as follows:
     * 
     * a = dv/dt
     * v = ds/dt => dt = ds/v
     * 
     * substituting dt:
     * 
     * a = v dv/ds
     * 
     * a = v (v0-v1)/ds
     * 
     * average v is (v0+v1)/2, so
     * 
     * a = (v0+v1)(v0-v1)/2ds
     * 
     * or
     * 
     * (v0^2 - v1^2)/2ds
     */
    private static double getActual_acceleration(
            ConstrainedState successor,
            ConstrainedState constraint_state,
            final double ds) {
        return (constraint_state.max_velocity * constraint_state.max_velocity
                - successor.max_velocity * successor.max_velocity) / (2.0 * ds);
    }

    /**
     * Clamp constraint state accelerations to the constraints.
     */
    private static void clampAccel(List<TimingConstraint> constraints, ConstrainedState constraint_state)
            throws TimingException {
        for (final TimingConstraint constraint : constraints) {
            final TimingConstraint.MinMaxAcceleration min_max_accel = constraint
                    .getMinMaxAcceleration(constraint_state.state, constraint_state.max_velocity);
            constraint_state.min_acceleration = Math.max(
                    constraint_state.min_acceleration,
                    min_max_accel.getMinAccel());
            constraint_state.max_acceleration = Math.min(
                    constraint_state.max_acceleration,
                    min_max_accel.getMaxAccel());
        }
        if (constraint_state.min_acceleration > constraint_state.max_acceleration) {
            throw new TimingException();
        }
    }

    /**
     * Integrate the constrained states forward in time to obtain the TimedStates.
     */
    private static Trajectory100 integrate(
            final List<Pose2dWithMotion> states,
            List<ConstrainedState> constraint_states) throws TimingException {
        List<TimedPose> timed_states = new ArrayList<>(states.size());
        double t = 0.0; // time along path
        double s = 0.0; // distance along path
        double v = 0.0;
        for (int i = 0; i < states.size(); ++i) {
            final ConstrainedState constrained_state = constraint_states.get(i);
            // Advance t.
            final double ds = constrained_state.distance - s;
            final double accel = (constrained_state.max_velocity * constrained_state.max_velocity - v * v) / (2.0 * ds);
            double dt = 0.0;
            if (i > 0) {
                timed_states.get(i - 1).set_acceleration(accel);
                dt = calculateDt(v, constrained_state, ds, accel);
            }
            t += dt;
            if (Double.isNaN(t) || Double.isInfinite(t)) {
                throw new TimingException();
            }

            v = constrained_state.max_velocity;
            s = constrained_state.distance;
            timed_states.add(
                    new TimedPose(constrained_state.state, t, v, accel));
        }
        return new Trajectory100(timed_states);
    }

    private static double calculateDt(double v, final ConstrainedState constrained_state, final double ds,
            final double accel) throws TimingException {
        if (Math.abs(accel) > kEpsilon) {
            return (constrained_state.max_velocity - v) / accel;
        } else if (Math.abs(v) > kEpsilon) {
            return ds / v;
        } else {
            throw new TimingException();
        }
    }

    protected static class ConstrainedState {
        public Pose2dWithMotion state;
        public double distance;
        public double max_velocity;
        public double min_acceleration;
        public double max_acceleration;

        @Override
        public String toString() {
            return state.toString() + ", distance: " + distance + ", max_velocity: " + max_velocity + ", " +
                    "min_acceleration: " + min_acceleration + ", max_acceleration: " + max_acceleration;
        }
    }

    public static class TimingException extends Exception {
    }

    private TimingUtil() {
    }
}