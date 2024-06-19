package org.team100.lib.controller;

import org.team100.lib.profile.Constraints100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

/**
 * A bang-bang controller with sigmoid effort at the switching point, to
 * eliminate chatter. The general idea is to move the zero-effort point away
 * from the switching point, and to reserve some extra effort in case we pass
 * the switching point. So there are three control effort levels: minimum,
 * target, and maximum.
 * 
 * This is a copy of TrapezoidProfile100, I need to add anti-chatter.
 * 
 * This uses the approach from LaValle 2023: between any two points in phase
 * space, the optimal acceleration-limited path is via two parabolas, perhaps
 * with a velocity limit.
 * 
 * The notation here is from that paper. Every point in phase space is
 * intersected with two minimum-time parabolas: one corresponding to maximum
 * acceleration, the other to maximum deceleration. The paper was concerned with
 * coordinating multiple axes, so there's a lot in it about non-minimum-time
 * paths. For our purpose here, the simple minimum-time case is all that
 * matters.
 * 
 * The simplest case links a state with zero velocity to another state with
 * position to the right, and also with zero velocity: the result is a
 * symmetrical pair of parabolas, an initial one for acceleration ("I" for
 * "initial") and the second for deceleration to the goal ("G" for "goal"). The
 * intersection is called the "switching point." A path with initial
 * acceleration followed by deceleration to the goal is called "I+G-". The other
 * pairing is also possible, for example the goal could be to the left of the
 * initial state, so the initial acceleration to get there would be negative.
 * 
 * The same picture applies to nonzero-velocity cases, and to cases with
 * velocities of opposite sign: any two states can be linked by either an I+G-
 * or I-G+ path.
 * 
 * Adding a velocity limit means that the "switching point" may be clipped off,
 * so the path is something like I+CG-, "C" for "cruise."
 * 
 * So there are three kinds of boundaries: I-G, I-cruise, and cruise-G.
 * 
 * https://arxiv.org/pdf/2210.01744.pdf
 * 
 * This class is different from the WPI profile class -- it keeps no state, it
 * just takes one dt step at a time. It doesn't have the notion of a profile
 * being "completed" -- if you want that, compare the output to the goal. The
 * main benefit of the approach here is that it correctly handles cases
 * involving moving end states that the WPI code does not handle correctly.
 * 
 * It might be slower around the switching points, since it can call itself once
 * or twice, once per segment.
 * 
 * 2024: to get this profile-maker to be a controller, add a linear part to the
 * sliding-mode.
 * 
 * the I and G curves are calculated using the supplied constraints, but the
 * applied effort is more, to push the system away from the boundary.
 */
public class BangBangController100 implements Profile100 {
    // extra effort at the boundary
    private static final double kExtra = 1.25;
    // half-width of the linear part of the response.
    private static final double kBoundarySec = 0.1;
    private final Constraints100 m_constraints;
    private final double m_tolerance;

    public BangBangController100(Constraints100 constraints, double tolerance) {
        m_constraints = constraints;
        m_tolerance = tolerance;
    }

    public BangBangController100(double maxVel, double maxAccel, double tolerance) {
        this(new Constraints100(maxVel, maxAccel), tolerance);
    }

    /**
     * Note order of the arguments: initial state first, then goal.
     * 
     * Input velocities are clamped to the velocity constraint.
     * 
     * Input accelerations are ignored: jerk is unmanaged.
     * 
     * Output acceleration is the *profile* acceleration at dt, it is not
     * necessarily the same as the actuation required to reach the output state from
     * the initial state. The only difference occurs when the dt period spans a
     * boundary between accel and decel, or accel/decel and cruise. In those cases,
     * the reported acceleration will be from the other side of the boundary.
     * 
     * Therefore, if you blindly use the reported acceleration as the only input to
     * a real system, it will tend to get ahead of the profile, but only by one time
     * period.
     */
    @Override
    public State100 calculate(double dt, final State100 initialRaw, final State100 goalRaw) {
        State100 initial = new State100(initialRaw.x(),
                MathUtil.clamp(initialRaw.v(), -m_constraints.maxVelocity, m_constraints.maxVelocity));
        State100 goal = new State100(goalRaw.x(),
                MathUtil.clamp(goalRaw.v(), -m_constraints.maxVelocity, m_constraints.maxVelocity));

        // AT THE GOAL

        if (goal.near(initial, m_tolerance)) {
            return goal;
        }

        // AT CRUISING VELOCITY

        if (MathUtil.isNear(m_constraints.maxVelocity, initial.v(), 1e-12)) {
            return keepCruising(dt, initial, goal);
        }
        if (MathUtil.isNear(-m_constraints.maxVelocity, initial.v(), 1e-12)) {
            return keepCruisingMinus(dt, initial, goal);
        }

        // Calculate the ETA to each switch point, or NaN if there's no valid path.
        double t1IplusGminus = t1IplusGminus(initial, goal);
        double t1IminusGplus = t1IminusGplus(initial, goal);

        if (Double.isNaN(t1IminusGplus) && Double.isNaN(t1IplusGminus)) {
            Util.warn("Both I-G+ and I+G- are NaN, this should never happen");
            return initial;
        }

        // ON THE INITIAL PATH

        if (Double.isNaN(t1IplusGminus)) {
            // the valid path is I-G+, assume we're on I-
            System.out.printf("on I- %6.3f\n", t1IminusGplus);
            return handleIminus(dt, initial, goal, t1IminusGplus);
        }

        if (Double.isNaN(t1IminusGplus)) {
            // the valid path is I+G-, assume we're on I+
            System.out.printf("on I+ %6.3f\n", t1IplusGminus);
            return handleIplus(dt, initial, goal, t1IplusGminus);
        }

        // ON THE GOAL PATH

        // There can be one path with zero duration, indicating that we're on the goal
        // path at the switch point. In that case, we want to switch immediately and
        // proceed to the goal.
        dt = truncateDt(dt, initial, goal);
        if (MathUtil.isNear(0, t1IminusGplus, 1e-12)) {
            return full(dt, initial, 1);
        }
        if (MathUtil.isNear(0, t1IplusGminus, 1e-12)) {
            return full(dt, initial, -1);
        }

        System.out.println("nonzero");

        // There can be two non-zero-duration paths. As above, this happens when we're
        // on the goal path. The difference is that in this case, the goal has non-zero
        // velocity. One path goes directly to the goal, but it's also possible to make
        // a little loop in phase space, backing up and ending up in the same place, on
        // the way to the goal. We want to avoid these little loops.
        if (t1IminusGplus > t1IplusGminus) {
            return full(dt, initial, 1);
        }
        return full(dt, initial, -1);
    }

    /** @param t1 time to switching, always positive */
    private State100 handleIplus(double dt, State100 initial, State100 goal, double t1) {
        if (t1 < kBoundarySec) {
            // within the boundary layer, so "soft switch"
            // truncateDt does nothing if we're outside dt.
            double effort = 1 - t1 / kBoundarySec;
            return full(truncateDt(dt, initial, goal), initial, -1.0 * effort);
        }
        if (t1 < (2.0 * kBoundarySec)) {
            // within the "slow down" boundary layer, so moderate the effort
            // truncateDt does nothing if we're outside dt.
            double effort = (t1 - kBoundarySec) / kBoundarySec;
            return full(truncateDt(dt, initial, goal), initial, effort);
        }
        if (MathUtil.isNear(t1, 0, 1e-12)) {
            // switch eta is zero: go to the goal via G-
            return full(truncateDt(dt, initial, goal), initial, -1);
        }
        if (t1 < dt) {
            // We Encounter G- during dt, so switch.
            return traverseSwitch(dt, initial, goal, t1, 1);
        }
        if (initial.v() + m_constraints.maxAcceleration * dt > m_constraints.maxVelocity) {
            // We encounter vmax, so cruise.
            return cruise(dt, initial, 1);
        }
        // We will not encounter any boundary during dt
        return full(dt, initial, 1);
    }

    /** @param t1 time to switching, sec, always positive */
    private State100 handleIminus(double dt, State100 initial, State100 goal, double t1) {
        if (t1 < kBoundarySec) {
            // within the boundary layer, so "soft switch"
            // truncateDt does nothing if we're outside dt.
            double effort = 1 - t1 / kBoundarySec;
            return full(truncateDt(dt, initial, goal), initial, effort);
        }
        if (t1 < (2.0 * kBoundarySec)) {
            // within the "slow down" boundary layer, so moderate the effort
            // truncateDt does nothing if we're outside dt.
            double effort = (t1 - kBoundarySec) / kBoundarySec;
            return full(truncateDt(dt, initial, goal), initial, -1.0 * effort);
        }
        if (MathUtil.isNear(t1, 0, 1e-12)) {
            // Switch ETA is zero: go to the goal via G+
            return full(truncateDt(dt, initial, goal), initial, 1);
        }
        if (t1 < dt) {
            // We encounter G+ during dt, so switch.
            return traverseSwitch(dt, initial, goal, t1, -1);
        }
        if (initial.v() - m_constraints.maxAcceleration * dt < -m_constraints.maxVelocity) {
            // we did encounter vmax, though
            return cruise(dt, initial, -1);
        }
        // We will not encounter any boundary during dt
        return full(dt, initial, -1);
    }

    private State100 keepCruising(double dt, State100 initial, State100 goal) {
        // We're already at positive cruising speed, which means G- is next.
        // will we reach it during dt?
        double c_minus = c_minus(goal);
        // the G- value at vmax
        double gminus = c_minus - Math.pow(m_constraints.maxVelocity, 2) / (2 * m_constraints.maxAcceleration);
        // distance to go
        double dc = gminus - initial.x();
        // time to go
        double dct = dc / m_constraints.maxVelocity;
        if (MathUtil.isNear(0, dct, 1e-12)) {
            // we are at the intersection of vmax and G-, so head down G-
            return full(truncateDt(dt, initial, goal), initial, -1);
        }
        if (dct < dt) {
            // there are two segments
            double tremaining = dt - dct;
            return calculate(tremaining, new State100(gminus, m_constraints.maxVelocity), goal);
        }
        // we won't reach G-, so cruise for all of dt.
        return new State100(
                initial.x() + m_constraints.maxVelocity * dt,
                m_constraints.maxVelocity,
                0);
    }

    private State100 keepCruisingMinus(double dt, State100 initial, State100 goal) {
        // We're already at negative cruising speed, which means G+ is next.
        // will we reach it during dt?
        double c_plus = c_plus(goal);
        double gplus = c_plus + Math.pow(m_constraints.maxVelocity, 2) / (2 * m_constraints.maxAcceleration);
        // negative
        double dc = gplus - initial.x();
        double dct = dc / -m_constraints.maxVelocity;
        if (MathUtil.isNear(0, dct, 1e-12)) {
            // We're at the intersection of -vmax and G+, so head up G+
            return full(truncateDt(dt, initial, goal), initial, 1);
        }
        if (dct < dt) {
            double tremaining = dt - dct;
            return calculate(tremaining, new State100(gplus, -m_constraints.maxVelocity), goal);
        }
        // we won't reach G+, so cruise for all of dt
        return new State100(
                initial.x() - m_constraints.maxVelocity * dt,
                -m_constraints.maxVelocity,
                0);
    }

    /**
     * Travel to the switching point, and then the remainder of time on the goal
     * path.
     */
    private State100 traverseSwitch(double dt, State100 in_initial, final State100 goal, double t1, double direction) {
        // first get to the switching point
        double x = in_initial.x() + in_initial.v() * t1
                + 0.5 * direction * m_constraints.maxAcceleration * Math.pow(t1, 2);
        double v = in_initial.v() + direction * m_constraints.maxAcceleration * t1;
        // then go the other way for the remaining time
        double t2 = dt - t1;
        // just use the same method for the second part
        // note this is slower than the code below so maybe put it back
        return calculate(t2, new State100(x, v), goal);
    }

    /** Returns a shorter dt to avoid overshooting the goal state. */
    private double truncateDt(double dt, State100 in_initial, State100 in_goal) {
        double dtg = Math.abs((in_initial.v() - in_goal.v()) / m_constraints.maxAcceleration);
        return Math.min(dt, dtg);
    }

    /**
     * Return dt at full accel.
     * 
     * direction is now actually "error"
     * 
     * This is one method for both I and G, which means I is faster than planned, so
     * the whole path is faster than planned, which I think is fine, we don't use
     * the ETA anywhere. Note if we want to coordinate multiple axes using ETA's
     * we'll have to change this.
     */
    private State100 full(double dt, State100 in_initial, double direction) {
        // final double scale = 0.1;
        direction = MathUtil.clamp(direction, -1, 1);
        double x_i = in_initial.x();
        double v_i = in_initial.v();
        double a = direction * m_constraints.maxAcceleration * kExtra;
        double v = v_i + a * dt;
        double x = x_i + v_i * dt + 0.5 * a * Math.pow(dt, 2);
        return new State100(x, v, a);
    }

    /**
     * The path contains an I-cruise boundary, so proceed in I to the boundary and
     * then at the cruise speed for the remaining time.
     */
    private State100 cruise(double dt, State100 in_initial, double direction) {
        // need to clip (this is negative)
        double dv = direction * m_constraints.maxVelocity - in_initial.v();
        // time to get to limit (positive)
        double vt = dv / (direction * m_constraints.maxAcceleration);
        // location of that limit
        double xt = in_initial.x() + in_initial.v() * vt
                + 0.5 * direction * m_constraints.maxAcceleration * Math.pow(vt, 2);
        // remaining time
        double vt2 = dt - vt;
        // during that time, do we hit G+? i think it's not possible,
        // because this is the "not switching" branch.
        // so we just move along it
        double x = xt + direction * m_constraints.maxVelocity * vt2;
        return new State100(x, direction * m_constraints.maxVelocity, 0);
    }

    /** Time to switch point for I+G- path, or NaN if there is no path. */
    double t1IplusGminus(State100 initial, State100 goal) {
        double q_dot_switch = qDotSwitchIplusGminus(initial, goal);
        // this fixes rounding errors
        if (MathUtil.isNear(initial.v(), q_dot_switch, 1e-6))
            return 0;
        double t1 = (q_dot_switch - initial.v()) / m_constraints.maxAcceleration;
        if (t1 < 0) {
            return Double.NaN;
        }
        return t1;
    }

    /** Time to switch point for I-G+ path, or NaN if there is no path. */
    double t1IminusGplus(State100 initial, State100 goal) {
        double q_dot_switch = qDotSwitchIminusGplus(initial, goal);
        // this fixes rounding errors
        if (MathUtil.isNear(initial.v(), q_dot_switch, 1e-6))
            return 0;

        double t1 = (q_dot_switch - initial.v()) / (-1.0 * m_constraints.maxAcceleration);
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
    double qDotSwitchIplusGminus(State100 initial, State100 goal) {
        if (initial.equals(goal))
            return initial.v();

        // intercept of I-
        double c_minus = c_minus(initial);
        // intercept of I+
        double c_plus = c_plus(initial);
        // position of I- at the velocity of goal
        double p_minus = c_minus - Math.pow(goal.v(), 2) / (2 * m_constraints.maxAcceleration);
        double p_plus = c_plus + Math.pow(goal.v(), 2) / (2 * m_constraints.maxAcceleration);

        // "limit" path we don't want.
        if (goal.v() <= initial.v() && goal.x() < p_minus)
            return Double.NaN;
        if (goal.v() > initial.v() && goal.x() < p_plus)
            return Double.NaN;

        // progress along I+
        double d = qSwitchIplusGminus(initial, goal) - c_plus(initial);
        // prevent rounding errors
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
    double qDotSwitchIminusGplus(State100 initial, State100 goal) {
        if (initial.equals(goal))
            return goal.v();

        // intercept of I-
        double c_minus = c_minus(initial);
        // intercept of I+
        double c_plus = c_plus(initial);
        // position of I- at the velocity of goal
        double p_minus = c_minus - Math.pow(goal.v(), 2) / (2 * m_constraints.maxAcceleration);
        double p_plus = c_plus + Math.pow(goal.v(), 2) / (2 * m_constraints.maxAcceleration);

        // "limit" path we don't want.

        if (goal.v() <= initial.v() && goal.x() > p_minus)
            return Double.NaN;
        if (goal.v() > initial.v() && goal.x() > p_plus)
            return Double.NaN;

        // progress along I-
        double d = qSwitchIminusGplus(initial, goal) - c_plus(goal);
        // prevent rounding errors
        if (d < 0)
            d = 0;

        return -1.0 * Math.sqrt(2 * m_constraints.maxAcceleration * d);
    }

    /**
     * Position at the midpoint between the intercepts of the positive-acceleration
     * path through the initial state and the negative-acceleration path through the
     * goal state, i.e. the I+G- path.
     */
    double qSwitchIplusGminus(State100 initial, State100 goal) {
        return (c_plus(initial) + c_minus(goal)) / 2;
    }

    /**
     * Midpoint position for the I-G+ path.
     */
    double qSwitchIminusGplus(State100 initial, State100 goal) {
        return (c_minus(initial) + c_plus(goal)) / 2;
    }

    /** Intercept of negative-acceleration path intersecting s */
    double c_minus(State100 s) {
        return s.x() - Math.pow(s.v(), 2) / (-2.0 * m_constraints.maxAcceleration);
    }

    /** Intercept of negative-acceleration path intersecting s */
    double c_plus(State100 s) {
        return s.x() - Math.pow(s.v(), 2) / (2.0 * m_constraints.maxAcceleration);
    }

    // for testing
    double t1(State100 initial, State100 goal) {
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
