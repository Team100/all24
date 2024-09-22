package org.team100.lib.controller;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.StringSupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

/**
 * This is a minimum-time controller for the double integrator, with
 * acceleration and velocity limits. The control policy is to apply maximum
 * acceleration towards the goal (tracing the "initial path" in state space, a
 * parabola), until the "switching curve" is reached, and then apply maximum
 * deceleration before arriving (following the switching curve to the goal, on
 * the "goal path"). This strategy is a type of "sliding mode" controller, and
 * so it is subject to "chatter" on the switching curve.
 * 
 * To prevent chattering, the switching curve acceleration is slightly higher
 * than the actual goal path acceleration, which leads the system to drift away
 * from the switching curve. The "initial path" acceleration is a little higher
 * than the switching curve acceleration, which pushes the system back on to the
 * switching curve. So the control output does chatter a bit between "really all
 * the way on" and "most of the way on" -- much better than full scale reverse
 * chatter. The level of chattering (and thus the level of disturbance-rejecting
 * control authority available) is adjustable. Note that these differences in
 * effort will make the ETA wrong, which is fine, we don't use the ETA anywhere.
 * Note if we want to coordinate multiple axes using ETA's, we'll have to change
 * this.
 * 
 * Control switches to full-state proportional feedback in the neighborhood of
 * the goal, to prevent full-scale orbiting.
 * 
 * The usual WPI pattern for this sort of path would be to construct a feasible
 * trapezoid profile, and then use a combination of feedforward and feedback
 * control to follow it. The issue with that approach is that the profile
 * blindly follows its own timer, which results in two problems: ignorance of
 * disturbances and tuning complexity: feedback parameters need to work well
 * with a very strong feedforward input *and* with the total absence of
 * feedforward (in case the profile timer expires).
 * 
 * {@see https://liberzon.csl.illinois.edu/teaching/cvoc/node85.html}
 * {@see https://underactuated.mit.edu/dp.html#minimum_time_double_integrator}
 * 
 * This class is very similar to
 * {@link org.team100.lib.profile.TrapezoidProfile100}, described here:
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
 * TODO: allow different acceleration and deceleration.
 */
public class MinTimeController implements Glassy {

    // how close to a boundary (e.g. switching curve, max v) to behave as if we were
    // "on" the boundary
    private static final double kBoundaryTolerance = 1e-12;

    // ANTI-CHATTER
    // switching curve is calculated using the supplied constraint
    // actual G effort is a little *less* than this constraint
    private final double m_weakG;
    // actual I effort is a little *more* than this constraint
    private final double m_strongI;
    // the result should be oscillation just past the switching curve, and no
    // reversing chatter

    // FULL STATE MODE NEAR THE GOAL
    // threshold below which we use proportional feedback.
    private final double m_finish;
    // low gains => low output, settles very slowly.
    // high gains => full output, settles very fast.
    // using high gain with delay will yield orbiting
    private final double[] m_k;

    private final DoubleUnaryOperator m_modulus;

    private final double m_maxVelocity;
    private final double m_switchingAcceleration;
    private final double m_tolerance;

    private final StringSupplierLogger2 m_log_mode;

    /**
     * @param modulus        for angle wrapping
     * @param maxVel
     * @param switchingAccel acceleration of the switching curve.
     * @param weakG          acceleration of the goal path, should be less than
     *                       switching.
     * @param strongI        acceleration of the initial path, should be more than
     *                       switching.
     * @param tolerance      when x and v are both closer than this, return the
     *                       goal.
     * @param finish         when x and v are closer than this, switch to
     *                       proportional
     *                       full-state
     * @param k              full-state gains
     */
    public MinTimeController(
            SupplierLogger2 parent,
            DoubleUnaryOperator modulus,
            double maxVel,
            double switchingAccel,
            double weakG,
            double strongI,
            double tolerance,
            double finish,
            double[] k) {
        m_modulus = modulus;
        m_maxVelocity = maxVel;
        m_switchingAcceleration = switchingAccel;
        m_weakG = weakG;
        m_strongI = strongI;
        m_tolerance = tolerance;
        m_finish = finish;
        m_k = k;
        SupplierLogger2 child = parent.child(this);
        m_log_mode = child.stringLogger(Level.TRACE, "mode");
    }

    private State100 modulus(double x, double v, double a) {
        return new State100(m_modulus.applyAsDouble(x), v, a);
    }

    private State100 modulus(State100 s) {
        return modulus(s.x(), s.v(), s.a());
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
    public State100 calculate(double dt, final State100 initialRaw, final State100 goalRaw) {
        State100 initial = new State100(initialRaw.x(),
                MathUtil.clamp(initialRaw.v(), -m_maxVelocity, m_maxVelocity));

        // for periodic state spaces, choose an equivalent goal close to the initial
        // state (may be outside the valid range, we'll fix it later).
        State100 goal = new State100(
                m_modulus.applyAsDouble(goalRaw.x() - initialRaw.x()) + initialRaw.x(),
                MathUtil.clamp(goalRaw.v(), -m_maxVelocity, m_maxVelocity));

        // AT THE GOAL: DO NOTHING
        if (goal.near(initial, m_tolerance)) {
            m_log_mode.log(() -> "within tolerance");
            return modulus(goal);
        }

        // NEAR THE GOAL: USE FULL STATE to avoid oscillation
        if (goal.near(initial, m_finish)) {
            m_log_mode.log(() -> "full state");
            double xError = goal.x() - initial.x();
            double vError = goal.v() - initial.v();
            double u_FBx = xError * m_k[0];
            double u_FBv = vError * m_k[1];
            double u_FB = u_FBx + u_FBv;
            double a = u_FB;
            double v = initial.v() + a * dt;
            double x = initial.x() + initial.v() * dt + 0.5 * a * Math.pow(dt, 2);
            return modulus(x, v, a);
        }

        // AT CRUISING VELOCITY

        if (MathUtil.isNear(m_maxVelocity, initial.v(), kBoundaryTolerance)) {
            return keepCruising(dt, initial, goal);
        }
        if (MathUtil.isNear(-m_maxVelocity, initial.v(), kBoundaryTolerance)) {
            return keepCruisingMinus(dt, initial, goal);
        }

        // Calculate the ETA to each switch point, or NaN if there's no valid path.
        double t1IplusGminus = t1IplusGminus(initial, goal);
        double t1IminusGplus = t1IminusGplus(initial, goal);

        if (Double.isNaN(t1IminusGplus) && Double.isNaN(t1IplusGminus)) {
            Util.warn("Both I-G+ and I+G- are NaN, this should never happen");
            return modulus(initial);
        }

        m_log_mode.log(() -> "min time");

        // ON THE INITIAL PATH

        if (Double.isNaN(t1IplusGminus)) {
            // the valid path is I-G+, assume we're on I-
            // System.out.printf("on I- %6.3f\n", t1IminusGplus);
            return handleIminus(dt, initial, goal, t1IminusGplus);
        }

        if (Double.isNaN(t1IminusGplus)) {
            // the valid path is I+G-, assume we're on I+
            // System.out.printf("on I+ %6.3f\n", t1IplusGminus);
            return handleIplus(dt, initial, goal, t1IplusGminus);
        }

        // ON THE GOAL PATH

        // There can be one path with zero duration, indicating that we're on the goal
        // path at the switch point. In that case, we want to switch immediately and
        // proceed to the goal.
        dt = truncateDt(dt, initial, goal);
        if (MathUtil.isNear(0, t1IminusGplus, kBoundaryTolerance)) {
            return fullG(dt, initial, 1);
        }
        if (MathUtil.isNear(0, t1IplusGminus, kBoundaryTolerance)) {
            return fullG(dt, initial, -1);
        }

        // System.out.println("nonzero");

        // There can be two non-zero-duration paths. As above, this happens when we're
        // on the goal path. The difference is that in this case, the goal has non-zero
        // velocity. One path goes directly to the goal, but it's also possible to make
        // a little loop in phase space, backing up and ending up in the same place, on
        // the way to the goal. We want to avoid these little loops.
        if (t1IminusGplus > t1IplusGminus) {
            return fullG(dt, initial, 1);
        }
        return fullG(dt, initial, -1);
    }

    /**
     * On the I path, what should we do?
     * 
     * @param t1 time to switching, always positive
     */
    private State100 handleIplus(double dt, State100 initial, State100 goal, double t1) {
        if (MathUtil.isNear(t1, 0, kBoundaryTolerance)) {
            // switch eta is zero! Switch to G, i.e. go to the goal via G-
            return fullG(truncateDt(dt, initial, goal), initial, -1);
        }
        if (t1 < dt) {
            // We Encounter G- during dt, so switch.
            return traverseSwitch(dt, initial, goal, t1, 1);
        }
        if (initial.v() + m_switchingAcceleration * dt > m_maxVelocity) {
            // We encounter vmax, so cruise.
            return cruise(dt, initial, 1);
        }
        // We will not encounter any boundary during dt, stay on I
        return fullI(dt, initial, 1);
    }

    /**
     * On the I path, what should we do?
     * 
     * @param t1 time to switching, sec, always positive
     */
    private State100 handleIminus(double dt, State100 initial, State100 goal, double t1) {
        if (MathUtil.isNear(t1, 0, kBoundaryTolerance)) {
            // Switch ETA is zero! Switch to G, i.e. go to the goal via G+
            return fullG(truncateDt(dt, initial, goal), initial, 1);
        }
        if (t1 < dt) {
            // We encounter G+ during dt, so switch.
            return traverseSwitch(dt, initial, goal, t1, -1);
        }
        if (initial.v() - m_switchingAcceleration * dt < -m_maxVelocity) {
            // we did encounter vmax, though
            return cruise(dt, initial, -1);
        }
        // We will not encounter any boundary during dt, stay on I
        return fullI(dt, initial, -1);
    }

    private State100 keepCruising(double dt, State100 initial, State100 goal) {
        // We're already at positive cruising speed, which means G- is next.
        // will we reach it during dt?
        double c_minus = c_minus(goal);
        // the G- value at vmax
        double gminus = c_minus - Math.pow(m_maxVelocity, 2) / (2 * m_switchingAcceleration);
        // distance to go
        double dc = gminus - initial.x();
        // time to go
        double dct = dc / m_maxVelocity;
        if (MathUtil.isNear(0, dct, kBoundaryTolerance)) {
            // we are at the intersection of vmax and G-, so head down G-
            return fullG(truncateDt(dt, initial, goal), initial, -1);
        }
        if (dct < dt) {
            // there are two segments
            double tremaining = dt - dct;
            return calculate(tremaining, new State100(gminus, m_maxVelocity), goal);
        }
        // we won't reach G-, so cruise for all of dt.
        return modulus(
                initial.x() + m_maxVelocity * dt,
                m_maxVelocity,
                0);
    }

    private State100 keepCruisingMinus(double dt, State100 initial, State100 goal) {
        // We're already at negative cruising speed, which means G+ is next.
        // will we reach it during dt?
        double c_plus = c_plus(goal);
        double gplus = c_plus + Math.pow(m_maxVelocity, 2) / (2 * m_switchingAcceleration);
        // negative
        double dc = gplus - initial.x();
        double dct = dc / -m_maxVelocity;
        if (MathUtil.isNear(0, dct, kBoundaryTolerance)) {
            // We're at the intersection of -vmax and G+, so head up G+
            return fullG(truncateDt(dt, initial, goal), initial, 1);
        }
        if (dct < dt) {
            double tremaining = dt - dct;
            return calculate(tremaining, new State100(gplus, -m_maxVelocity), goal);
        }
        // we won't reach G+, so cruise for all of dt
        return modulus(
                initial.x() - m_maxVelocity * dt,
                -m_maxVelocity,
                0);
    }

    /**
     * Travel to the switching point, and then the remainder of time on the goal
     * path.
     */
    private State100 traverseSwitch(double dt, State100 in_initial, final State100 goal, double t1, double direction) {
        // first get to the switching point
        double x = in_initial.x() + in_initial.v() * t1
                + 0.5 * direction * m_switchingAcceleration * Math.pow(t1, 2);
        double v = in_initial.v() + direction * m_switchingAcceleration * t1;
        // then go the other way for the remaining time
        double t2 = dt - t1;
        // just use the same method for the second part
        // note this is slower than the code below so maybe put it back
        return calculate(t2, new State100(x, v), goal);
    }

    /** Returns a shorter dt to avoid overshooting the goal state. */
    private double truncateDt(double dt, State100 in_initial, State100 in_goal) {
        double dtg = Math.abs((in_initial.v() - in_goal.v()) / m_switchingAcceleration);
        return Math.min(dt, dtg);
    }

    /**
     * For I paths, use slightly-stronger effort.
     */
    private State100 fullI(double dt, State100 in_initial, double direction) {
        // final double scale = 0.1;
        direction = MathUtil.clamp(direction, -1, 1);
        double x_i = in_initial.x();
        double v_i = in_initial.v();
        double a = direction * m_strongI;
        double v = v_i + a * dt;
        double x = x_i + v_i * dt + 0.5 * a * Math.pow(dt, 2);
        return modulus(x, v, a);
    }

    /**
     * For G paths, use slightly-weaker effort.
     */
    private State100 fullG(double dt, State100 in_initial, double direction) {
        // final double scale = 0.1;
        direction = MathUtil.clamp(direction, -1, 1);
        double x_i = in_initial.x();
        double v_i = in_initial.v();
        double a = direction * m_weakG;
        double v = v_i + a * dt;
        double x = x_i + v_i * dt + 0.5 * a * Math.pow(dt, 2);
        return modulus(x, v, a);
    }

    /**
     * The path contains an I-cruise boundary, so proceed in I to the boundary and
     * then at the cruise speed for the remaining time.
     */
    private State100 cruise(double dt, State100 in_initial, double direction) {
        // need to clip (this is negative)
        double dv = direction * m_maxVelocity - in_initial.v();
        // time to get to limit (positive)
        double vt = dv / (direction * m_switchingAcceleration);
        // location of that limit
        double xt = in_initial.x() + in_initial.v() * vt
                + 0.5 * direction * m_switchingAcceleration * Math.pow(vt, 2);
        // remaining time
        double vt2 = dt - vt;
        // during that time, do we hit G+? i think it's not possible,
        // because this is the "not switching" branch.
        // so we just move along it
        double x = xt + direction * m_maxVelocity * vt2;
        return modulus(x, direction * m_maxVelocity, 0);
    }

    /** Time to switch point for I+G- path, or NaN if there is no path. */
    double t1IplusGminus(State100 initial, State100 goal) {
        double q_dot_switch = qDotSwitchIplusGminus(initial, goal);
        // this fixes rounding errors
        if (MathUtil.isNear(initial.v(), q_dot_switch, 1e-6))
            return 0;
        double t1 = (q_dot_switch - initial.v()) / m_switchingAcceleration;
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

        double t1 = (q_dot_switch - initial.v()) / (-1.0 * m_switchingAcceleration);
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
        double p_minus = c_minus - Math.pow(goal.v(), 2) / (2 * m_switchingAcceleration);
        double p_plus = c_plus + Math.pow(goal.v(), 2) / (2 * m_switchingAcceleration);

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
        return Math.sqrt(2 * m_switchingAcceleration * d);
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
        double p_minus = c_minus - Math.pow(goal.v(), 2) / (2 * m_switchingAcceleration);
        double p_plus = c_plus + Math.pow(goal.v(), 2) / (2 * m_switchingAcceleration);

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

        return -1.0 * Math.sqrt(2 * m_switchingAcceleration * d);
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
        return s.x() - Math.pow(s.v(), 2) / (-2.0 * m_switchingAcceleration);
    }

    /** Intercept of negative-acceleration path intersecting s */
    double c_plus(State100 s) {
        return s.x() - Math.pow(s.v(), 2) / (2.0 * m_switchingAcceleration);
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

    @Override
    public String getGlassName() {
        return "MinTimeController";
    }
}
