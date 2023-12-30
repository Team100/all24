package org.team100.lib.profile;

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

    /**
     * True if the most-recent input to calculate indicates completion.
     */
    private boolean m_finished;

    public TrapezoidProfile100(Constraints constraints, double tolerance) {
        m_constraints = constraints;
        m_tolerance = tolerance;
    }

    public State calculate(double dt, final State in_goal, final State in_initial) {

        return null;
    }


    double tSwitch(State initial, State goal) {
        double tIplusGminus = tSwitchIplusGminus(initial, goal);
        double tIminusGplus = tSwitchIminusGplus(initial, goal);
        return Math.min(tIplusGminus, tIminusGplus);
    }
    double tSwitchIplusGminus(State initial, State goal) {
        // this is the switching velocity
        double q_dot_switch = qDotSwitchIplusGminus( initial, goal);
        double t_1 = (q_dot_switch - initial.velocity) / m_constraints.maxAcceleration;
        double t_2 = (goal.velocity - q_dot_switch) / (-1.0 * m_constraints.maxAcceleration);
        return t_1 + t_2;
    }
    double tSwitchIminusGplus(State initial, State goal) {
        double q_dot_switch = qDotSwitchIminusGplus(initial, goal);
        double t_1 = (q_dot_switch - initial.velocity) / (-1.0 * m_constraints.maxAcceleration);
        double t_2 = (goal.velocity - q_dot_switch) / m_constraints.maxAcceleration;
        return t_1 + t_2;
    }

    double qDotSwitchIplusGminus(State initial, State goal) {
        return Math.sqrt(2 * m_constraints.maxAcceleration * (qSwitchIplusGminus(initial, goal) - c_plus(initial)));    
    }

    double qDotSwitchIminusGplus(State initial, State goal) {
        return -1.0 * Math.sqrt(
            2 * m_constraints.maxAcceleration * (qSwitchIminusGplus(initial, goal) - c_plus(goal)));
    }

    double qSwitchIplusGminus(State initial, State goal) {
        return (c_plus(initial) + c_minus(goal)) / 2;
    }

    double qSwitchIminusGplus(State initial, State goal) {
        return (c_minus(initial) + c_plus(goal))/2;
    }

    /** Intercept of negative-acceleration path intersecting s */
    double c_minus(State s) {
        return s.position - Math.pow(s.velocity,2)/(-2.0 * m_constraints.maxAcceleration);
    }

    /** Intercept of negative-acceleration path intersecting s */
    double c_plus(State s) {
        return s.position - Math.pow(s.velocity, 2)/(2.0 * m_constraints.maxAcceleration);
    }




    public State calculate2(double dt, final State in_goal, final State in_current) {
        m_finished = false;
        double u = u(dt, in_goal, in_current);
        if (m_finished)
            return in_goal;
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
            m_finished = true;
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

    /**
     * True if the most recent input to calculate indicates completion.
     */
    public boolean isFinished() {
        return m_finished;
    }
}
