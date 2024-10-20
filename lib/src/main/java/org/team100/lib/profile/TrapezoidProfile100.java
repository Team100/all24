package org.team100.lib.profile;

import org.team100.lib.state.State100;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

/**
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
 * Oct 3 2024, a new requirement is to compute the profile duration, for the
 * purpose of coordinating multiple profiles. In general, this sort of
 * coordination is complicated, involving three different "switching point"
 * possibilities, but for the goal-at-rest case, there's just one ("x_switch"
 * from the paper) and i think you can just scale the constraints. For an
 * implementation of the general case, see team100/studies2023/rrt, specifically
 * the RRTStar classes, which include coordination in their path solvers.
 */
public class TrapezoidProfile100 implements Profile100 {
    private static final boolean DEBUG = false;

    private final double m_maxVelocity;
    private final double m_maxAcceleration;
    private final double m_tolerance;

    public TrapezoidProfile100(double maxVel, double maxAccel, double tolerance) {
        m_maxVelocity = maxVel;
        m_maxAcceleration = maxAccel;
        m_tolerance = tolerance;
    }

    public TrapezoidProfile100 scale(double s) {
        return new TrapezoidProfile100(m_maxVelocity, s * m_maxAcceleration, m_tolerance);
    }

    public double solve(double dt, State100 i, State100 g, double eta, double etaTolerance) {
        return solveForSlowerETA(
                m_maxVelocity,
                m_maxAcceleration,
                m_tolerance,
                dt,
                i,
                g,
                eta,
                etaTolerance);
    }

    /**
     * Return scale factor to make the ETA equal to the desired ETA, by reducing
     * acceleration.
     * 
     * It never returns s > 1, and it also never scales more than 10X, i.e. never
     * returns s < 0.01.
     * 
     * It is very approximate, in order to not run too long. It's very primitive.
     */
    public static double solveForSlowerETA(
            double maxV,
            double maxA,
            double tol,
            double dt,
            State100 initial,
            State100 goal,
            double eta,
            double sTolerance) {
        final double minS = 0.01;
        final double maxS = 1.0;
        return Math100.findRoot(
                s -> getEtaS(maxV, maxA, tol, dt, initial, goal, eta, s),
                minS,
                getEtaS(maxV, maxA, tol, dt, initial, goal, eta, minS),
                maxS,
                getEtaS(maxV, maxA, tol, dt, initial, goal, eta, maxS),
                sTolerance, 100);
    }

    private static double getEtaS(
            double maxV,
            double maxA,
            double tol,
            double dt,
            State100 initial,
            State100 goal,
            double eta, double s) {
        TrapezoidProfile100 p = new TrapezoidProfile100(
                maxV,
                s * maxA,
                tol);
        ResultWithETA r = p.calculateWithETA(dt, initial, goal);
        double etaS = r.etaS();
        return etaS - eta;
    }

    @Override
    public State100 calculate(double dt, State100 initial, State100 goal) {
        return calculateWithETA(dt, initial, goal).state();
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
     * 
     * Returns the goal if the intial is within tolerance of it.
     */
    @Override
    public ResultWithETA calculateWithETA(double dt, final State100 initialRaw, final State100 goalRaw) {
        // Too-high initial speed is handled with braking
        if (initialRaw.v() > m_maxVelocity) {
            if (DEBUG)
                Util.printf("positive entry speed too fast, braking %s\n", initialRaw.toString());
            State100 result = full(dt, initialRaw, -1);
            double extraSpeed = initialRaw.v() - m_maxVelocity;
            double extraTime = extraSpeed / m_maxAcceleration;
            double brakingDistance = 0.5 * extraSpeed * extraTime + m_maxVelocity * extraTime;
            State100 afterBraking = new State100(initialRaw.x() + brakingDistance, m_maxVelocity, -m_maxAcceleration);
            ResultWithETA remainder = calculateWithETA(dt, afterBraking, goalRaw);
            return new ResultWithETA(result, extraTime + remainder.etaS());
        }
        if (initialRaw.v() < -m_maxVelocity) {
            if (DEBUG)
                Util.printf("negative entry speed too fast, braking %s\n", initialRaw.toString());
            State100 result = full(dt, initialRaw, 1);
            // this is a positive number
            double extraSpeed = -1.0 * initialRaw.v() - m_maxVelocity;
            double extraTime = extraSpeed / m_maxAcceleration;
            double brakingDistance = 0.5 * extraSpeed * extraTime + m_maxVelocity * extraTime;
            State100 afterBraking = new State100(initialRaw.x() - brakingDistance, -m_maxVelocity, m_maxAcceleration);
            ResultWithETA remainder = calculateWithETA(dt, afterBraking, goalRaw);
            return new ResultWithETA(result, extraTime + remainder.etaS());
        }
        State100 initial = initialRaw;
        // Too-high goal speed is not allowed
        if (goalRaw.v() > m_maxVelocity || goalRaw.v() < -m_maxVelocity) {
            Util.warn("Goal velocity is higher than profile velocity");
        }
        State100 goal = limitVelocity(goalRaw);

        if (goal.near(initial, m_tolerance)) {
            if (DEBUG)
                Util.printf("at goal\n");
            return new ResultWithETA(goal, 0);
        }

        // Calculate the ETA to each switch point, or NaN if there's no valid path.
        double t1IplusGminus = t1IplusGminus(initial, goal);
        double t1IminusGplus = t1IminusGplus(initial, goal);

        if (Double.isNaN(t1IminusGplus) && Double.isNaN(t1IplusGminus)) {
            Util.warn("Both I-G+ and I+G- are NaN, this should never happen");
            return new ResultWithETA(initial, 0);
        }

        if (Double.isNaN(t1IplusGminus)) {
            // t1IminusGplus is ok
            // the valid path is I-G+, assume we're on I-
            if (DEBUG)
                Util.printf("assume we're on I-\n");
            return handleIminus(dt, initial, goal, t1IminusGplus);
        }

        if (Double.isNaN(t1IminusGplus)) {
            // t1IplusGminus is ok
            // the valid path is I+G-, assume we're on I+
            if (DEBUG)
                Util.printf("assume we're on I+\n");
            return handleIplus(dt, initial, goal, t1IplusGminus);
        }

        // if we got here, we're on the goal path, so the remaining time is at full
        // accel
        double duration = durationAtMaxA(initial.v(), goal.v());

        // There can be one path with zero duration, indicating that we're on the goal
        // path at the switch point. In that case, we want to switch immediately and
        // proceed to the goal.
        dt = truncateDt(dt, initial, goal);
        if (MathUtil.isNear(0, t1IminusGplus, 1e-12)) {
            if (DEBUG)
                Util.printf("assume we're on G+\n");
            return new ResultWithETA(full(dt, initial, 1), duration);
        }
        if (MathUtil.isNear(0, t1IplusGminus, 1e-12)) {
            if (DEBUG)
                Util.printf("assume we're on G-\n");
            return new ResultWithETA(full(dt, initial, -1), duration);
        }

        // There can be two non-zero-duration paths. As above, this happens when we're
        // on the goal path. The difference is that in this case, the goal has non-zero
        // velocity. One path goes directly to the goal, but it's also possible to make
        // a little loop in phase space, backing up and ending up in the same place, on
        // the way to the goal. We want to avoid these little loops.
        if (t1IminusGplus > t1IplusGminus) {
            if (DEBUG)
                Util.printf("we're on G+\n");
            return new ResultWithETA(full(dt, initial, 1), duration);
        }
        if (DEBUG)
            Util.printf("we're on G-\n");
        return new ResultWithETA(full(dt, initial, -1), duration);
    }

    /** Clamp state velocity to the profile limit. */
    private State100 limitVelocity(final State100 s) {
        return new State100(
                s.x(),
                MathUtil.clamp(s.v(), -m_maxVelocity, m_maxVelocity),
                s.a());
    }

    private ResultWithETA handleIplus(double dt, State100 initial, State100 goal, double timeToSwitch) {
        if (MathUtil.isNear(timeToSwitch, 0, 1e-12)) {
            // switch eta is zero: go to the goal via G-
            if (DEBUG)
                Util.printf("at switch, go to G-\n");
            double duration = durationAtMaxA(initial.v(), goal.v());
            return new ResultWithETA(full(truncateDt(dt, initial, goal), initial, -1), duration);
        }
        // how much time to get to cruise? (remember initial v is positive)
        double timeToCruise = (m_maxVelocity - initial.v()) / m_maxAcceleration;

        if (timeToSwitch < dt && timeToSwitch < timeToCruise) {
            // We Encounter G- during dt, before cruise, so switch.
            if (DEBUG)
                Util.printf("switch is soon\n");
            return traverseSwitch(dt, initial, goal, timeToSwitch, 1);
        }

        if (timeToCruise < dt) {
            // we encounter vmax during dt.
            // what's the switching position?
            double x = initial.x()
                    + initial.v() * timeToCruise
                    + 0.5 * m_maxAcceleration * Math.pow(timeToCruise, 2);
            State100 nextState = new State100(x, m_maxVelocity, m_maxAcceleration);
            ResultWithETA r = keepCruising(dt - timeToCruise, nextState, goal);
            // return the next state with the total ETA.
            return new ResultWithETA(r.state(), timeToCruise + r.etaS());
        }
        // We will not encounter any boundary during dt
        State100 result = full(dt, initial, 1);

        // but we still need to know the full duration.
        //
        // the two possibilities are I+G- and I+C+G-
        if (timeToSwitch < timeToCruise) {
            // we hit the switching point first
            // so if we proceed for t1, what velocity will we be at?
            if (DEBUG)
                Util.printf("no cruise\n");
            State100 switchPoint = full(timeToSwitch, initial, 1);
            double durationG = durationAtMaxA(switchPoint.v(), goal.v());
            return new ResultWithETA(result, timeToSwitch + durationG);
        }
        // we hit cruise first
        if (DEBUG)
            Util.printf("go cruise\n");
        State100 cruisePoint = full(timeToCruise, initial, 1);
        ResultWithETA remainingCruise = keepCruising(dt, cruisePoint, goal);
        return new ResultWithETA(result, timeToCruise + remainingCruise.etaS());

    }

    /**
     * t1 is the time to the I-G+ switch point, which might be beyond the velocity
     * constraint.
     */
    private ResultWithETA handleIminus(double dt, State100 initial, State100 goal, double timeToSwitch) {

        if (MathUtil.isNear(timeToSwitch, 0, 1e-12)) {
            // Switch ETA is zero: go to the goal via G+
            double duration = durationAtMaxA(initial.v(), goal.v());
            return new ResultWithETA(full(truncateDt(dt, initial, goal), initial, 1), duration);
        }
        // how much time to get to cruise? (remember initial v is negative)
        double timeToCruise = (m_maxVelocity + initial.v()) / m_maxAcceleration;

        if (timeToSwitch < dt && timeToSwitch < timeToCruise) {
            // We encounter G+ during dt, so switch.
            return traverseSwitch(dt, initial, goal, timeToSwitch, -1);
        }
        if (timeToCruise < dt) {
            // we encounter vmax during dt.
            // what's the switching position?
            double x = initial.x()
                    + initial.v() * timeToCruise
                    - 0.5 * m_maxAcceleration * Math.pow(timeToCruise, 2);
            State100 nextState = new State100(x, -m_maxVelocity, -m_maxAcceleration);
            ResultWithETA r = keepCruisingMinus(dt - timeToCruise, nextState, goal);
            return new ResultWithETA(r.state(), timeToCruise + r.etaS());
        }
        // We will not encounter any boundary during dt, so the resulting state is just
        // "full throttle for dt"
        State100 result = full(dt, initial, -1);

        // but we still need to know the full duration.
        //
        // the two possibilities are I-G+ and I-C-G+
        if (timeToSwitch < timeToCruise) {
            // we hit the switching point first
            // so if we proceed for t1, what velocity will we be at?
            State100 switchPoint = full(timeToSwitch, initial, -1);
            double durationG = durationAtMaxA(switchPoint.v(), goal.v());
            return new ResultWithETA(result, timeToSwitch + durationG);
        }
        // we hit cruise first
        State100 cruisePoint = full(timeToCruise, initial, -1);
        ResultWithETA remainingCruise = keepCruisingMinus(dt, cruisePoint, goal);
        return new ResultWithETA(result, timeToCruise + remainingCruise.etaS());
    }

    /** At positive cruising speed, keep going. */
    ResultWithETA keepCruising(double dt, State100 initial, State100 goal) {
        if (DEBUG)
            Util.printf("keep cruising %s\n", initial);

        // We're already at positive cruising speed, which means G- is next.
        // will we reach it during dt?
        // this is the x location of the v0 intercept of the goal path
        double c_minus = c_minus(goal);
        if (DEBUG)
            Util.printf("c_minus %6.3f\n", c_minus);
        // the G- value at current velocity (vmax)
        double gminus = c_minus - Math.pow(m_maxVelocity, 2) / (2 * m_maxAcceleration);
        // distance to go to the G- intersection
        double dc = gminus - initial.x();
        // time to go to the G- intersection
        double durationToGMinus = dc / m_maxVelocity;
        if (MathUtil.isNear(0, durationToGMinus, 1e-12)) {
            // we are at the intersection of vmax and G-, so head down G-
            State100 result = full(truncateDt(dt, initial, goal), initial, -1);
            // on the goal path so the remaining duration is maxA.
            double duration = durationAtMaxA(initial.v(), goal.v());
            return new ResultWithETA(result, duration);
        }
        double durationFromGMinusToGoal = durationAtMaxA(m_maxVelocity, goal.v());
        if (durationToGMinus < dt) {
            // we reach G- before the end of dt, so we spend part of the time
            // getting there, and the remaining time going down G-. we want
            // to return the end state on G-
            double tremaining = dt - durationToGMinus;
            if (DEBUG)
                Util.printf("tremaining %6.3f\n", tremaining);
            ResultWithETA r = calculateWithETA(tremaining, new State100(gminus, m_maxVelocity), goal);
            // the total ETA is the time to G-, plus the time *on* G-
            return new ResultWithETA(r.state(), durationToGMinus + r.etaS());
        }
        // we won't reach G-, so cruise for all of dt.
        return new ResultWithETA(new State100(
                initial.x() + m_maxVelocity * dt,
                m_maxVelocity,
                0),
                durationToGMinus + durationFromGMinusToGoal);
    }

    ResultWithETA keepCruisingMinus(double dt, State100 initial, State100 goal) {
        // We're already at negative cruising speed, which means G+ is next.
        // will we reach it during dt?
        double c_plus = c_plus(goal);
        double gplus = c_plus + Math.pow(m_maxVelocity, 2) / (2 * m_maxAcceleration);
        // negative
        double dc = gplus - initial.x();
        // time to go to the G+ intersection
        double durationToGPlus = dc / -m_maxVelocity;
        if (MathUtil.isNear(0, durationToGPlus, 1e-12)) {
            // We're at the intersection of -vmax and G+, so head up G+
            State100 result = full(truncateDt(dt, initial, goal), initial, 1);
            // on the goal path so the remaining duration is maxA.
            double duration = durationAtMaxA(initial.v(), goal.v());
            return new ResultWithETA(result, duration);
        }
        double durationFromGPlusToGoal = durationAtMaxA(m_maxVelocity, goal.v());
        if (durationToGPlus < dt) {
            double tremaining = dt - durationToGPlus;
            ResultWithETA r = calculateWithETA(tremaining, new State100(gplus, -m_maxVelocity), goal);
            return new ResultWithETA(r.state(), durationToGPlus + r.etaS());
        }
        // we won't reach G+, so cruise for all of dt
        return new ResultWithETA(new State100(
                initial.x() - m_maxVelocity * dt,
                -m_maxVelocity,
                0),
                durationToGPlus + durationFromGPlusToGoal);
    }

    /**
     * Travel to the switching point, and then the remainder of time on the goal
     * path.
     */
    private ResultWithETA traverseSwitch(double dt, State100 in_initial, final State100 goal, double t1,
            double direction) {
        if (DEBUG)
            Util.printf("traverse switch\n");
        // first get to the switching point
        double x = in_initial.x() + in_initial.v() * t1
                + 0.5 * direction * m_maxAcceleration * Math.pow(t1, 2);
        double v = in_initial.v() + direction * m_maxAcceleration * t1;
        // then go the other way for the remaining time
        double t2 = dt - t1;
        // just use the same method for the second part
        // note this is slower than the code below so maybe put it back
        State100 s = new State100(x, v);
        if (DEBUG)
            Util.printf("switch state %s\n", s);

        ResultWithETA r = calculateWithETA(t2, s, goal);
        return new ResultWithETA(r.state(), t1 + r.etaS());
    }

    /** Returns a shorter dt to avoid overshooting the goal state. */
    private double truncateDt(double dt, State100 in_initial, State100 in_goal) {
        double dtg = durationAtMaxA(in_initial.v(), in_goal.v());
        return Math.min(dt, dtg);
    }

    /**
     * duration of the maximum-acceleration path from initial velocity to goal
     * velocity. this is the duration of the *actual* path from initial to goal, if
     * they lie on the same constant-acceleration parabola.
     */
    private double durationAtMaxA(double v_i, double v_g) {
        return Math.abs((v_i - v_g) / m_maxAcceleration);
    }

    /**
     * Extrapolate from the initial state at full acceleration for the duration dt.
     * This function is unaware of the goal per se; call it when you know what
     * direction to go and when you're sure you can proceed for the whole dt time
     * period.
     */
    private State100 full(double dt, State100 in_initial, double direction) {
        if (DEBUG)
            Util.printf("full x_i %s\n", in_initial);
        double x = in_initial.x() + in_initial.v() * dt
                + 0.5 * direction * m_maxAcceleration * Math.pow(dt, 2);
        double v = in_initial.v() + direction * m_maxAcceleration * dt;
        double a = direction * m_maxAcceleration;
        return new State100(x, v, a);
    }

    /**
     * Time to switch point for I+G- path, or NaN if there is no path.
     * 
     * Note this ignores the velocity constraint.
     */
    double t1IplusGminus(State100 initial, State100 goal) {
        double q_dot_switch = qDotSwitchIplusGminus(initial, goal);
        // this fixes rounding errors
        if (MathUtil.isNear(initial.v(), q_dot_switch, 1e-6))
            return 0;
        double t1 = (q_dot_switch - initial.v()) / m_maxAcceleration;
        if (t1 < 0) {
            return Double.NaN;
        }
        return t1;
    }

    /**
     * Time to switch point for I-G+ path, or NaN if there is no path.
     * 
     * Note this ignores the velocity constraint.
     */
    double t1IminusGplus(State100 initial, State100 goal) {
        double q_dot_switch = qDotSwitchIminusGplus(initial, goal);
        // this fixes rounding errors
        if (MathUtil.isNear(initial.v(), q_dot_switch, 1e-6))
            return 0;

        double t1 = (q_dot_switch - initial.v()) / (-1.0 * m_maxAcceleration);
        if (t1 < 0) {
            return Double.NaN;
        }
        return t1;
    }

    /**
     * Velocity of I+ at the switching point of the "switch" path.
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
        double p_minus = c_minus - Math.pow(goal.v(), 2) / (2 * m_maxAcceleration);
        double p_plus = c_plus + Math.pow(goal.v(), 2) / (2 * m_maxAcceleration);

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
        return Math.sqrt(2 * m_maxAcceleration * d);
    }

    /**
     * Velocity of G+ at the switching point of the "switch" path.
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
        double p_minus = c_minus - Math.pow(goal.v(), 2) / (2 * m_maxAcceleration);
        double p_plus = c_plus + Math.pow(goal.v(), 2) / (2 * m_maxAcceleration);

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

        return -1.0 * Math.sqrt(2 * m_maxAcceleration * d);
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
        return s.x() - Math.pow(s.v(), 2) / (-2.0 * m_maxAcceleration);
    }

    /** Intercept of positive-acceleration path intersecting s */
    double c_plus(State100 s) {
        return s.x() - Math.pow(s.v(), 2) / (2.0 * m_maxAcceleration);
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

    public double getMaxVelocity() {
        return m_maxVelocity;
    }

    public double getMaxAcceleration() {
        return m_maxAcceleration;
    }

    public double getTolerance() {
        return m_tolerance;
    }
}
