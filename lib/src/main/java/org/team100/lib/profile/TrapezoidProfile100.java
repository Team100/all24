package org.team100.lib.profile;

import edu.wpi.first.math.MathUtil;

/**
 * This uses the approach from LaValle 2023: between any two points in phase
 * space, the optimal acceleration-limited path is via two parabolas, perhaps
 * with a velocity limit.
 *
 * 
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

        if (Double.isNaN(initial.position))
            throw new IllegalArgumentException();
        if (Double.isNaN(initial.velocity))
            throw new IllegalArgumentException();
        if (Double.isNaN(goal.position))
            throw new IllegalArgumentException();
        if (Double.isNaN(goal.velocity))
            throw new IllegalArgumentException();

        State in_initial = new State(initial.position,
                MathUtil.clamp(initial.velocity, -m_constraints.maxVelocity, m_constraints.maxVelocity));
        State in_goal = new State(goal.position,
                MathUtil.clamp(goal.velocity, -m_constraints.maxVelocity, m_constraints.maxVelocity));

        if (in_goal.near(in_initial, m_tolerance)) {
            return new State(in_goal);
        }

        if (MathUtil.isNear(m_constraints.maxVelocity, in_initial.velocity, 1e-12)) {
            // cruising +x which means G- is next
            // will we reach it during dt?
            double c_minus = c_minus(in_goal);
            // the G- value at vmax
            double gminus = c_minus - Math.pow(m_constraints.maxVelocity, 2) / (2 * m_constraints.maxAcceleration);
            // distance to go
            double dc = gminus - in_initial.position;
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
                return new State(in_initial.position + m_constraints.maxVelocity * dt, m_constraints.maxVelocity);
            }

        } else if (MathUtil.isNear(-m_constraints.maxVelocity, in_initial.velocity, 1e-12)) {
            // cruising -x which means G+ is next
            // same logic as above, inverted.
            double c_plus = c_plus(in_goal);
            double gplus = c_plus + Math.pow(m_constraints.maxVelocity, 2) / (2 * m_constraints.maxAcceleration);
            // negative
            double dc = gplus - in_initial.position;
            double dct = dc / -m_constraints.maxVelocity;
            if (MathUtil.isNear(0, dct, 1e-12)) {
                // we're on G+ already, fall through
            } else if (dct < dt) {
                double tremaining = dt - dct;
                return calculate(tremaining, new State(gplus, -m_constraints.maxVelocity), in_goal);
            } else {
                // we won't reach G+
                return new State(in_initial.position - m_constraints.maxVelocity * dt, -m_constraints.maxVelocity);

            }
        }

        double t1IplusGminus = t1IplusGminus(in_initial, in_goal);
        double t1IminusGplus = t1IminusGplus(in_initial, in_goal);

        if (Double.isNaN(t1IminusGplus) && Double.isNaN(t1IplusGminus))
            throw new IllegalStateException();

        if (Double.isNaN(t1IplusGminus)) {
            // the valid path is I-G+
            double t1 = t1IminusGplus;

            if (MathUtil.isNear(t1, 0, 1e-12)) {
                // we're on G+
                // time to get to the goal, positive
                double dtg = Math.abs((in_initial.velocity - in_goal.velocity) / m_constraints.maxAcceleration);
                // don't overshoot the goal
                dt = Math.min(dt, dtg);
                double x = in_initial.position + in_initial.velocity * dt
                        + 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
                double v = in_initial.velocity + m_constraints.maxAcceleration * dt;
                return new State(x, v);
            } else if (t1 >= dt) {
                // we're on I- the whole dt duration

                double v = in_initial.velocity - m_constraints.maxAcceleration * dt;

                if (v < -m_constraints.maxVelocity) {
                    // need to clip (this is negative)
                    double dv = -m_constraints.maxVelocity - in_initial.velocity;
                    // time to get to limit (positive)
                    double vt = dv / -m_constraints.maxAcceleration;
                    // location of that limit
                    double xt = in_initial.position + in_initial.velocity * vt
                            - 0.5 * m_constraints.maxAcceleration * Math.pow(vt, 2);
                    // remaining time
                    double vt2 = dt - vt;
                    // during that time, do we hit G+? i think it's not possible,
                    // because this is the "not switching" branch.
                    // so we just move along it
                    double x = xt - m_constraints.maxVelocity * vt2;
                    return new State(x, -m_constraints.maxVelocity);
                }

                double x = in_initial.position + in_initial.velocity * dt
                        - 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
                return new State(x, v);
            }
            // switch during dt
            // first get to the switching point
            double x = in_initial.position + in_initial.velocity * t1
                    - 0.5 * m_constraints.maxAcceleration * Math.pow(t1, 2);
            double v = in_initial.velocity - m_constraints.maxAcceleration * t1;
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
                double dtg = Math.abs((in_initial.velocity - in_goal.velocity) / m_constraints.maxAcceleration);
                // don't overshoot the goal
                dt = Math.min(dt, dtg);
                double x = in_initial.position + in_initial.velocity * dt
                        - 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
                double v = in_initial.velocity - m_constraints.maxAcceleration * dt;
                return new State(x, v);

            } else if (t1 >= dt) {
                // we're on I+ the whole dt duration

                double v = in_initial.velocity + m_constraints.maxAcceleration * dt;

                if (v > m_constraints.maxVelocity) {
                    // need to clip
                    double dv = m_constraints.maxVelocity - in_initial.velocity;
                    // time to get to limit
                    double vt = dv / m_constraints.maxAcceleration;
                    // location of that limit
                    double xt = in_initial.position + in_initial.velocity * vt
                            + 0.5 * m_constraints.maxAcceleration * Math.pow(vt, 2);
                    // remaining time
                    double vt2 = dt - vt;
                    // during that time, do we hit G-? i think it's not possible,
                    // because this is the "not switching" branch.
                    // so we just move along it
                    double x = xt + m_constraints.maxVelocity * vt2;
                    return new State(x, m_constraints.maxVelocity);
                }
                double x = in_initial.position + in_initial.velocity * dt
                        + 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
                return new State(x, v);
            }
            // switch during dt
            // first get to the switching point
            double x = in_initial.position + in_initial.velocity * t1
                    + 0.5 * m_constraints.maxAcceleration * Math.pow(t1, 2);
            double v = in_initial.velocity + m_constraints.maxAcceleration * t1;
            // then go the other way for the remaining time
            double t2 = dt - t1;
            return calculate(t2, new State(x, v), goal);

        }
        // neither is NaN.

        // time to get to the goal, positive
        double dtg = Math.abs((in_initial.velocity - in_goal.velocity) / m_constraints.maxAcceleration);
        // don't overshoot the goal
        dt = Math.min(dt, dtg);
        if (MathUtil.isNear(0, t1IminusGplus, 1e-12)) {
            // we want G+, use positive accel
            double x = in_initial.position + in_initial.velocity * dt
                    + 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
            double v = in_initial.velocity + m_constraints.maxAcceleration * dt;
            return new State(x, v);
        }
        if (MathUtil.isNear(0, t1IplusGminus, 1e-12)) {
            // we want G-, use negative accel
            double x = in_initial.position + in_initial.velocity * dt
                    - 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
            double v = in_initial.velocity - m_constraints.maxAcceleration * dt;
            return new State(x, v);
        }
        // if either path will work, but neither is zero, take the fast one.
        // chose the slower one
        if (t1IminusGplus > t1IplusGminus) {
            // I-G+ is slower so use I+G-, which means positive A
            double x = in_initial.position + in_initial.velocity * dt
                    + 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
            double v = in_initial.velocity + m_constraints.maxAcceleration * dt;
            return new State(x, v);
        }
        // I+G- is slower so use I-G+, which means negative A
        double x = in_initial.position + in_initial.velocity * dt
                - 0.5 * m_constraints.maxAcceleration * Math.pow(dt, 2);
        double v = in_initial.velocity - m_constraints.maxAcceleration * dt;
        return new State(x, v);
    }

    // for testing
    double t1(State initial, State goal) {
        double t1IplusGminus = t1IplusGminus(initial, goal);
        double t1IminusGplus = t1IminusGplus(initial, goal);
        return t1(t1IplusGminus, t1IminusGplus);
    }

    double t1(double t1IplusGminus, double t1IminusGplus) {
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

    double tSwitchIplusGminus(State initial, State goal) {
        // this is the switching velocity
        double q_dot_switch = qDotSwitchIplusGminus(initial, goal);
        double t_1 = t1IplusGminus(initial, goal);
        double t_2 = (goal.velocity - q_dot_switch) / (-1.0 * m_constraints.maxAcceleration);
        return t_1 + t_2;
    }

    /** Time to switch point for I+G- path, or NaN if there is no path. */
    double t1IplusGminus(State initial, State goal) {
        double q_dot_switch = qDotSwitchIplusGminus(initial, goal);
        // this fixes rounding errors
        if (MathUtil.isNear(initial.velocity, q_dot_switch, 1e-6))
            return 0;
        double t1 = (q_dot_switch - initial.velocity) / m_constraints.maxAcceleration;
        if (t1 < 0) {
            return Double.NaN;
        }
        return t1;
    }

    double tSwitchIminusGplus(State initial, State goal) {
        double q_dot_switch = qDotSwitchIminusGplus(initial, goal);
        double t_1 = t1IminusGplus(initial, goal);
        double t_2 = (goal.velocity - q_dot_switch) / m_constraints.maxAcceleration;
        return t_1 + t_2;
    }

    /** Time to switch point for I-G+ path, or NaN if there is no path. */
    double t1IminusGplus(State initial, State goal) {
        double q_dot_switch = qDotSwitchIminusGplus(initial, goal);
        // this fixes rounding errors
        if (MathUtil.isNear(initial.velocity, q_dot_switch, 1e-6))
            return 0;

        double t1 = (q_dot_switch - initial.velocity) / (-1.0 * m_constraints.maxAcceleration);
        if (t1 < 0) {
            return Double.NaN;
        }
        return t1;
    }

    /**
     * Velocity of I+ at the midpoint of the "switch" path.
     * The "switch" path always chooses a switching point that is faster than the
     * endpoints. If there's no intersection, or if the intersection corresponds to
     * a "limit" path, you get NaN.
     * 
     * "switch" path using I+G- means the goal has to be to the right of the "s"
     * shaped curve including I.
     */
    double qDotSwitchIplusGminus(State initial, State goal) {
        if (initial.equals(goal))
            return initial.velocity;

        // intercept of I-
        double c_minus = c_minus(initial);
        // intercept of I+
        double c_plus = c_plus(initial);
        // position of I- at the velocity of goal
        double p_minus = c_minus - Math.pow(goal.velocity, 2) / (2 * m_constraints.maxAcceleration);
        double p_plus = c_plus + Math.pow(goal.velocity, 2) / (2 * m_constraints.maxAcceleration);

        // "limit" path we don't want.

        if (goal.velocity <= initial.velocity && goal.position < p_minus)
            return Double.NaN;
        if (goal.velocity > initial.velocity && goal.position < p_plus)
            return Double.NaN;

        // progress along I+
        double d = qSwitchIplusGminus(initial, goal) - c_plus(initial);
        if (d < 0)
            throw new IllegalStateException();
        double qdot = Math.sqrt(2 * m_constraints.maxAcceleration * d);
        // avoid the "tlimit" path
        // ok actually qdot doesn't need to be less than both, it just
        // needs to be greater than one.

        return qdot;

    }

    /**
     * Velocity of G+ at the midpoint of the "switch" path.
     * The "switch" path always chooses a switching point that is faster than the
     * endpoints. If there's no intersection, or if the intersection corresponds to
     * a "limit" path, you get NaN.
     * 
     * "switch" path using I-G+ means the goal has to be to the left of the I- curve
     * for goal.v less than i.v, and to the left of the I+ curve for goal.v > i.v
     */
    double qDotSwitchIminusGplus(State initial, State goal) {
        if (initial.equals(goal))
            return goal.velocity;

        // intercept of I-
        double c_minus = c_minus(initial);
        // intercept of I+
        double c_plus = c_plus(initial);
        // position of I- at the velocity of goal
        double p_minus = c_minus - Math.pow(goal.velocity, 2) / (2 * m_constraints.maxAcceleration);
        double p_plus = c_plus + Math.pow(goal.velocity, 2) / (2 * m_constraints.maxAcceleration);

        // "limit" path we don't want.

        if (goal.velocity <= initial.velocity && goal.position > p_minus)
            return Double.NaN;
        if (goal.velocity > initial.velocity && goal.position > p_plus)
            return Double.NaN;

        // progress along I-
        double d = qSwitchIminusGplus(initial, goal) - c_plus(goal);
        if (d < 0)
            throw new IllegalStateException();
        double qdot = Math.sqrt(2 * m_constraints.maxAcceleration * d);

        return -qdot;
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
        return s.position - Math.pow(s.velocity, 2) / (-2.0 * m_constraints.maxAcceleration);
    }

    /** Intercept of negative-acceleration path intersecting s */
    double c_plus(State s) {
        return s.position - Math.pow(s.velocity, 2) / (2.0 * m_constraints.maxAcceleration);
    }

    public State calculate2(double dt, final State in_goal, final State in_current) {
        double u = u(dt, in_goal, in_current);
        double v = in_current.velocity + u * dt;
        if (v > m_constraints.maxVelocity) {
            v = m_constraints.maxVelocity;
            u = 0;
        } else if (v < -m_constraints.maxVelocity) {
            v = -m_constraints.maxVelocity;
            u = 0;
        }
        double x = in_current.position + in_current.velocity * dt + 0.5 * u * dt * dt;
        // System.out.println("U " + u);
        return new State(x, v);
    }

    /** Produce +maxAccel, -maxAccel, or zero. */
    public double u(double dt, final State in_goal, final State in_current) {
        double gain = m_constraints.maxAcceleration;
        State error = in_goal.minus(in_current);
        // double err = error.norm();
        // if we're near the goal, turn down the gain in the 2nd and 4th quadrants.
        if ((Math.abs(error.position) < 5 * m_constraints.maxVelocity * dt)
                &&
                (Math.abs(error.velocity) < 5 * m_constraints.maxAcceleration * dt)
                &&
                ((in_current.position > in_goal.position && in_current.velocity < 0)
                        ||
                        (in_current.position < in_goal.position && in_current.velocity > 0))) {
            gain = 0.1 * gain;
        }
        // if (err < 10 * dt) {
        // System.out.println("slow");
        // gain = 0.1 * gain;
        // }
        // System.out.println("GOAL " + in_goal + " CURRENT " + in_current);
        if (in_goal.near(in_current, m_tolerance)) {
            return 0;
        }

        // the intercept of the forward parabola
        double xplus = xplus(in_goal, gain);
        // the intercept of the reverse parabola
        double xminus = xminus(in_goal, gain);
        // the value of the forward parabola at our velocity
        double xplusSwitch = xplusforv(xplus, in_current.velocity, gain);
        // the value of the reverse parabola at our velocity
        double xminusSwitch = xminusforv(xminus, in_current.velocity, gain);

        // System.out.printf("xp %5.3f xm %5.3f\n", xplusSwitch, xminusSwitch);

        // which side of the switching surface are we on?
        if (in_current.position < xminusSwitch) {
            // System.out.println("left");
            // to the left of the reverse parabola.

            if (in_current.position > xplusSwitch) {
                // System.out.println("center");
                // the region in the center. in this region we're too close to get directly to
                // the goal. instead, we back up to provide more space to accumluate speed.
                if (in_goal.velocity > 0) {
                    // if the goal velocity is positive, then backing is reverse.
                    // if we're close to the boundary, then coast
                    // use the coasting time since it's easier than calculating the parabola
                    // intersection
                    double coastTimeToBoundary = (in_current.position - xplusSwitch) / -in_current.velocity;
                    // System.out.println("time " + coastTimeToBoundary);
                    if (in_current.velocity < 0 && in_current.position - xplusSwitch < -3 * in_current.velocity * dt) {
                        // System.out.println("coast0");
                        // if the coast just barely intersects the switching surface, then we should
                        // switch at dt/2.
                        // and u = 0. so naively, use double the dt period.
                        return gain * (1 - coastTimeToBoundary / dt);
                        // return 0.0;
                    }
                    // otherwise back full speed.
                    return -gain;
                }
                // if the goal velocity is negative, then backing is forward.
                // if we're close to the boundary, then coast
                double coastTimeToBoundary = (xminusSwitch - in_current.position) / in_current.velocity;
                // System.out.println("lefttime " + coastTimeToBoundary);
                if (in_current.velocity > 0 && xminusSwitch - in_current.position < 3 * in_current.velocity * dt) {
                    // System.out.println("coast1");
                    return gain * (coastTimeToBoundary / dt - 1);
                    // return 0.0;
                }
                // otherwise ahead full
                return gain;
            }
            // the rest of the left region wants forward.
            // if we're close to the boundary, then coast
            double coastTimeToBoundary = (xminusSwitch - in_current.position) / in_current.velocity;
            // System.out.println("TIME " + coastTimeToBoundary);
            if (in_current.velocity > 0 && xminusSwitch - in_current.position < 3 * in_current.velocity * dt) {
                // System.out.println("coast2");
                // guess at a reasonable u. if the coast time is long, then this is +u, if the
                // coast time is short it's -u
                return gain * (coastTimeToBoundary / dt - 1);
                // return 0.0;
            }
            // otherwise ahead full

            return gain;
        }
        if (in_current.position > xplusSwitch) {
            // System.out.println("right");
            // to the right of the forward parabola. since we already accounted for the
            // overlap, we always reverse.
            // if we're close to the boundary, then coast
            double coastTimeToBoundary = (in_current.position - xplusSwitch) / -in_current.velocity;
            // System.out.println("Right time " + coastTimeToBoundary);
            if (in_current.velocity < 0 && in_current.position - xplusSwitch < -3 * in_current.velocity * dt) {
                // System.out.println("coast3");
                return gain * (1 - coastTimeToBoundary / dt);
                // return 0.0;
            }
            // otherwise back full speed.
            return -gain;
        }
        // within neither parabola, i.e. going too fast to stop, so make a u-turn.
        if (in_current.velocity > 0) {
            // System.out.println("u0");
            // if we're heading forward, we need to reverse
            return -gain;
        }
        // if we're heading reverse, we need to go forward
        // System.out.println("u1");
        return gain;
    }

    /**
     * positive parabola value
     * 
     * @param x0 x intercept
     * @param v  sample velocity
     * @return position of the parabola for the velocity
     */

    double xplusforv(double x0, double v, double gain) {
        return x0 + 0.5 * v * v / gain;
    }

    /**
     * reverse parabola value
     * 
     * @param x0 x intercept
     * @param v  sample velocity
     * @return position of the parabola for the velocity
     */
    double xminusforv(double x0, double v, double gain) {
        return x0 - 0.5 * v * v / gain;

    }

    /** The x intercept of the positive-going trajectory */
    double xplus(State s, double gain) {
        return s.position - 0.5 * s.velocity * s.velocity / gain;
    }

    /** The x intercept of the positive-going trajectory */
    double xminus(State s, double gain) {
        return s.position + 0.5 * s.velocity * s.velocity / gain;
    }
}
