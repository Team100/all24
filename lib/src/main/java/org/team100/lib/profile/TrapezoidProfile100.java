package org.team100.lib.profile;

import edu.wpi.first.math.MathUtil;

/**
 * This uses the approach from LaValle 2023: between any two points in phase
 * space, the optimal acceleration-limited path is via two parabolas, perhaps
 * with a velocity limit.
 */
public class TrapezoidProfile100 {

    private final Constraints m_constraints;
    private final double m_tolerance;

    public TrapezoidProfile100(Constraints constraints, double tolerance) {
        m_constraints = constraints;
        m_tolerance = tolerance;
    }

    /**
     * this puts initial first because my god
     * also clamps initial and goal velocities to the constraint.
     * the velocity constraint only ever clips the switch point.
     */
    public State calculate(double dt, final State initial, final State goal) {
        State in_initial = new State(initial.getPosition(),
                MathUtil.clamp(initial.getVelocity(), -m_constraints.maxVelocity, m_constraints.maxVelocity));
        State in_goal = new State(goal.getPosition(),
                MathUtil.clamp(goal.getVelocity(), -m_constraints.maxVelocity, m_constraints.maxVelocity));

        if (in_goal.near(in_initial, m_tolerance)) {
            return new State(in_goal);
        }

        if (MathUtil.isNear(m_constraints.maxVelocity, in_initial.getVelocity(), 1e-12)) {
            // cruising +x which means G- is next
            // will we reach it during dt?
            double c_minus = c_minus(in_goal);
            // the G- value at vmax
            double gminus = c_minus - Math.pow(m_constraints.maxVelocity, 2) / (2 * m_constraints.maxAcceleration);
            // distance to go
            double dc = gminus - in_initial.getPosition();
            // time to go
            double dct = dc / m_constraints.maxVelocity;
            if (MathUtil.isNear(0, dct, 1e-12)) {
                // we're on G- already
            } else if (dct < dt) {
                // there are two segments
                double tremaining = dt - dct;
                return calculate(tremaining, new State(gminus, m_constraints.maxVelocity), in_goal);
            } else {
                // we won't reach G-
                return new State(in_initial.getPosition() + m_constraints.maxVelocity * dt, m_constraints.maxVelocity);
            }

        } else if (MathUtil.isNear(-m_constraints.maxVelocity, in_initial.getVelocity(), 1e-12)) {
            // cruising -x which means G+ is next
            // same logic as above, inverted.
            double c_plus = c_plus(in_goal);
            double gplus = c_plus + Math.pow(m_constraints.maxVelocity, 2) / (2 * m_constraints.maxAcceleration);
            // negative
            double dc = gplus - in_initial.getPosition();
            double dct = dc / -m_constraints.maxVelocity;
            if (MathUtil.isNear(0, dct, 1e-12)) {
                // we're on G+ already, fall through
            } else if (dct < dt) {
                double tremaining = dt - dct;
                return calculate(tremaining, new State(gplus, -m_constraints.maxVelocity), in_goal);
            } else {
                // we won't reach G+
                return new State(in_initial.getPosition() - m_constraints.maxVelocity * dt, -m_constraints.maxVelocity);

            }
        }

        double t1IplusGminus = t1IplusGminus(in_initial, in_goal);
        double t1IminusGplus = t1IminusGplus(in_initial, in_goal);

        // these should not both be NaN
        if (Double.isNaN(t1IminusGplus) && Double.isNaN(t1IplusGminus))
            return in_initial;

        if (Double.isNaN(t1IplusGminus)) {
            // the valid path is I-G+
            double t1 = t1IminusGplus;

            if (MathUtil.isNear(t1, 0, 1e-12)) {
                // we're on G+
                // time to get to the goal, positive
                double dtg = Math.abs((in_initial.getVelocity() - in_goal.getVelocity()) / m_constraints.maxAcceleration);
                // don't overshoot the goal
                dt = Math.min(dt, dtg);
                double x = in_initial.getPosition() + in_initial.getVelocity() * dt
                        + 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
                double v = in_initial.getVelocity() + m_constraints.maxAcceleration * dt;
                return new State(x, v);
            } else if (t1 >= dt) {
                // we're on I- the whole dt duration

                double v = in_initial.getVelocity() - m_constraints.maxAcceleration * dt;

                if (v < -m_constraints.maxVelocity) {
                    // need to clip (this is negative)
                    double dv = -m_constraints.maxVelocity - in_initial.getVelocity();
                    // time to get to limit (positive)
                    double vt = dv / -m_constraints.maxAcceleration;
                    // location of that limit
                    double xt = in_initial.getPosition() + in_initial.getVelocity() * vt
                            - 0.5 * m_constraints.maxAcceleration * Math.pow(vt, 2);
                    // remaining time
                    double vt2 = dt - vt;
                    // during that time, do we hit G+? i think it's not possible,
                    // because this is the "not switching" branch.
                    // so we just move along it
                    double x = xt - m_constraints.maxVelocity * vt2;
                    return new State(x, -m_constraints.maxVelocity);
                }

                double x = in_initial.getPosition() + in_initial.getVelocity() * dt
                        - 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
                return new State(x, v);
            }
            // switch during dt
            // first get to the switching point
            double x = in_initial.getPosition() + in_initial.getVelocity() * t1
                    - 0.5 * m_constraints.maxAcceleration * Math.pow(t1, 2);
            double v = in_initial.getVelocity() - m_constraints.maxAcceleration * t1;
            // then go the other way for the remaining time
            double t2 = dt - t1;
            // just use the same method for the second part
            // note this is slower than the code below so maybe put it back
            return calculate(t2, new State(x, v), goal);
        }
        if (Double.isNaN(t1IminusGplus)) {
            // the valid path is I+G-
            double t1 = t1IplusGminus;
            if (MathUtil.isNear(t1, 0, 1e-12)) {
                // we're on G-
                // time to get to the goal, positive
                double dtg = Math.abs((in_initial.getVelocity() - in_goal.getVelocity()) / m_constraints.maxAcceleration);
                // don't overshoot the goal
                dt = Math.min(dt, dtg);
                double x = in_initial.getPosition() + in_initial.getVelocity() * dt
                        - 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
                double v = in_initial.getVelocity() - m_constraints.maxAcceleration * dt;
                return new State(x, v);

            } else if (t1 >= dt) {
                // we're on I+ the whole dt duration

                double v = in_initial.getVelocity() + m_constraints.maxAcceleration * dt;

                if (v > m_constraints.maxVelocity) {
                    // need to clip
                    double dv = m_constraints.maxVelocity - in_initial.getVelocity();
                    // time to get to limit
                    double vt = dv / m_constraints.maxAcceleration;
                    // location of that limit
                    double xt = in_initial.getPosition() + in_initial.getVelocity() * vt
                            + 0.5 * m_constraints.maxAcceleration * Math.pow(vt, 2);
                    // remaining time
                    double vt2 = dt - vt;
                    // during that time, do we hit G-? i think it's not possible,
                    // because this is the "not switching" branch.
                    // so we just move along it
                    double x = xt + m_constraints.maxVelocity * vt2;
                    return new State(x, m_constraints.maxVelocity);
                }
                double x = in_initial.getPosition() + in_initial.getVelocity() * dt
                        + 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
                return new State(x, v);
            }
            // switch during dt
            // first get to the switching point
            double x = in_initial.getPosition() + in_initial.getVelocity() * t1
                    + 0.5 * m_constraints.maxAcceleration * Math.pow(t1, 2);
            double v = in_initial.getVelocity() + m_constraints.maxAcceleration * t1;
            // then go the other way for the remaining time
            double t2 = dt - t1;
            return calculate(t2, new State(x, v), goal);

        }
        // neither is NaN.

        // time to get to the goal, positive
        double dtg = Math.abs((in_initial.getVelocity() - in_goal.getVelocity()) / m_constraints.maxAcceleration);
        // don't overshoot the goal
        dt = Math.min(dt, dtg);

        if (MathUtil.isNear(0, t1IminusGplus, 1e-12)) {
            // we want G+, use positive accel
            double x = in_initial.getPosition() + in_initial.getVelocity() * dt
                    + 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
            double v = in_initial.getVelocity() + m_constraints.maxAcceleration * dt;
            return new State(x, v);
        }
        if (MathUtil.isNear(0, t1IplusGminus, 1e-12)) {
            // we want G-, use negative accel
            double x = in_initial.getPosition() + in_initial.getVelocity() * dt
                    - 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
            double v = in_initial.getVelocity() - m_constraints.maxAcceleration * dt;
            return new State(x, v);
        }

        // if either path will work, but neither is zero, take the fast one.
        if (t1IminusGplus > t1IplusGminus) {
            // I-G+ is slower so use I+G-, which means positive A
            double x = in_initial.getPosition() + in_initial.getVelocity() * dt
                    + 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
            double v = in_initial.getVelocity() + m_constraints.maxAcceleration * dt;
            return new State(x, v);
        }
        // I+G- is slower so use I-G+, which means negative A
        double x = in_initial.getPosition() + in_initial.getVelocity() * dt
                - 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
        double v = in_initial.getVelocity() - m_constraints.maxAcceleration * dt;
        return new State(x, v);
    }

    double tSwitchIplusGminus(State initial, State goal) {
        // this is the switching velocity
        double q_dot_switch = qDotSwitchIplusGminus(initial, goal);
        double t_1 = t1IplusGminus(initial, goal);
        double t_2 = (goal.getVelocity() - q_dot_switch) / (-1.0 * m_constraints.maxAcceleration);
        return t_1 + t_2;
    }

    /** Time to switch point for I+G- path, or NaN if there is no path. */
    double t1IplusGminus(State initial, State goal) {
        double q_dot_switch = qDotSwitchIplusGminus(initial, goal);
        // this fixes rounding errors
        if (MathUtil.isNear(initial.getVelocity(), q_dot_switch, 1e-6))
            return 0;
        double t1 = (q_dot_switch - initial.getVelocity()) / m_constraints.maxAcceleration;
        if (t1 < 0) {
            return Double.NaN;
        }
        return t1;
    }

    double tSwitchIminusGplus(State initial, State goal) {
        double q_dot_switch = qDotSwitchIminusGplus(initial, goal);
        double t_1 = t1IminusGplus(initial, goal);
        double t_2 = (goal.getVelocity() - q_dot_switch) / m_constraints.maxAcceleration;
        return t_1 + t_2;
    }

    /** Time to switch point for I-G+ path, or NaN if there is no path. */
    double t1IminusGplus(State initial, State goal) {
        double q_dot_switch = qDotSwitchIminusGplus(initial, goal);
        // this fixes rounding errors
        if (MathUtil.isNear(initial.getVelocity(), q_dot_switch, 1e-6))
            return 0;

        double t1 = (q_dot_switch - initial.getVelocity()) / (-1.0 * m_constraints.maxAcceleration);
        if (t1 < 0) {
            return Double.NaN;
        }
        return t1;
    }

    /**
     * Velocity of I+ at the midpoint of the "switch" path.
     * 
     * "switch" path using I+G- means the goal has to be to the right of the "s"
     * shaped curve including I.
     */
    double qDotSwitchIplusGminus(State initial, State goal) {
        if (initial.equals(goal))
            return initial.getVelocity();

        // intercept of I-
        double c_minus = c_minus(initial);
        // intercept of I+
        double c_plus = c_plus(initial);
        // position of I- at the velocity of goal
        double p_minus = c_minus - Math.pow(goal.getVelocity(), 2) / (2 * m_constraints.maxAcceleration);
        double p_plus = c_plus + Math.pow(goal.getVelocity(), 2) / (2 * m_constraints.maxAcceleration);

        // "limit" path we don't want.

        if (goal.getVelocity() <= initial.getVelocity() && goal.getPosition() < p_minus)
            return Double.NaN;
        if (goal.getVelocity() > initial.getVelocity() && goal.getPosition() < p_plus)
            return Double.NaN;

        // progress along I+
        double d = qSwitchIplusGminus(initial, goal) - c_plus(initial);
        if (d < 0)
            d = 0;
        return Math.sqrt(2 * m_constraints.maxAcceleration * d);
    }

    /**
     * Velocity of G+ at the midpoint of the "switch" path.
     * 
     * "switch" path using I-G+ means the goal has to be to the left of the I- curve
     * for goal.v less than i.v, and to the left of the I+ curve for goal.v > i.v
     */
    double qDotSwitchIminusGplus(State initial, State goal) {
        if (initial.equals(goal))
            return goal.getVelocity();

        // intercept of I-
        double c_minus = c_minus(initial);
        // intercept of I+
        double c_plus = c_plus(initial);
        // position of I- at the velocity of goal
        double p_minus = c_minus - Math.pow(goal.getVelocity(), 2) / (2 * m_constraints.maxAcceleration);
        double p_plus = c_plus + Math.pow(goal.getVelocity(), 2) / (2 * m_constraints.maxAcceleration);

        // "limit" path we don't want.

        if (goal.getVelocity() <= initial.getVelocity() && goal.getPosition() > p_minus)
            return Double.NaN;
        if (goal.getVelocity() > initial.getVelocity() && goal.getPosition() > p_plus)
            return Double.NaN;

        // progress along I-
        double d = qSwitchIminusGplus(initial, goal) - c_plus(goal);
        if (d < 0)
            d = 0;

        return -1.0 * Math.sqrt(2 * m_constraints.maxAcceleration * d);
    }

    /**
     * Position at the midpoint between the intercepts of the positive-acceleration
     * path through the initial state and the negative-acceleration path through the
     * goal state, i.e. the I+G- path.
     */
    double qSwitchIplusGminus(State initial, State goal) {
        return (c_plus(initial) + c_minus(goal)) / 2;
    }

    /**
     * Midpoint position for the I-G+ path.
     */
    double qSwitchIminusGplus(State initial, State goal) {
        return (c_minus(initial) + c_plus(goal)) / 2;
    }

    /** Intercept of negative-acceleration path intersecting s */
    double c_minus(State s) {
        return s.getPosition() - Math.pow(s.getVelocity(), 2) / (-2.0 * m_constraints.maxAcceleration);
    }

    /** Intercept of negative-acceleration path intersecting s */
    double c_plus(State s) {
        return s.getPosition() - Math.pow(s.getVelocity(), 2) / (2.0 * m_constraints.maxAcceleration);
    }

    // for testing
    double t1(State initial, State goal) {
        double t1IplusGminus = t1IplusGminus(initial, goal);
        double t1IminusGplus = t1IminusGplus(initial, goal);
        if (Double.isNaN(t1IplusGminus)) {
            return t1IminusGplus;
        }
        if (Double.isNaN(t1IminusGplus)) {
            return t1IplusGminus;
        }
        // this only happens when the positions are the same and the velocities are
        // opposite, i.e. they're on the same trajectory, in which case we want to
        // switch immediately
        return 0;
    }
}
