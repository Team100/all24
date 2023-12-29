package org.team100.lib.profile;

/**
 * This is a copy of the WPILib TrapezoidProfile, with two changes.
 * 
 * 1. the deprecated API is removed.
 * 2. the acceleration is exposed, so that feedforward controllers can use it.
 * 3. removed timeLeftUntil() because i don't think it's very useful.
 * 4. moved Constraints and State to separate files, to declutter.
 * 
 * This class does the wrong thing if the initial state is heading for the goal
 * fast enough to produce overshoot.
 */
public class TrapezoidProfile100 {

    private final Constraints m_constraints;

    // The direction of the profile, either 1 for forwards or -1 for inverted
    private int m_direction;

    /**
     * State supplied most recently to calculate.
     */
    private State m_current;

    /**
     * Time of the end of the acceleration period.
     */
    private double m_endAccel;

    /**
     * Time of the end of the cruise period. The same as endAccel for
     * triangular profiles.
     */
    private double m_endFullSpeed;
    
    /**
     * Time of the end of the deceleration period; also the end of the profile.
     */
    private double m_endDeccel;

    private boolean m_finished;

    public TrapezoidProfile100(Constraints constraints) {
        m_constraints = constraints;
    }

    /**
     * Given a measurement and goal, calculate the state dt in the future, compliant
     * with the constraints.
     * 
     * Calculate the correct position and velocity for the profile at a time t where
     * the beginning of the profile was at time t = 0.
     *
     * @param dt      Time in the future the state is for.
     * @param goal    Desired end state.
     * @param current current state.
     */
    public State calculate(double dt, State goal, State current) {
        m_direction = shouldFlipAcceleration(current, goal) ? -1 : 1;
        m_current = direct(current);
        goal = direct(goal);

        if (m_current.velocity > m_constraints.maxVelocity) {
            m_current.velocity = m_constraints.maxVelocity;
        }

        // Deal with a possibly truncated motion profile (with nonzero initial or
        // final velocity) by calculating the parameters as if the profile began and
        // ended at zero velocity
        double cutoffBegin = m_current.velocity / m_constraints.maxAcceleration;
        double cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

        double cutoffEnd = goal.velocity / m_constraints.maxAcceleration;
        double cutoffDistEnd = cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one

        double fullTrapezoidDist = cutoffDistBegin + (goal.position - m_current.position) + cutoffDistEnd;
        double accelerationTime = m_constraints.maxVelocity / m_constraints.maxAcceleration;

        double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxAcceleration;

        // Handle the case where the profile never reaches full speed
        if (fullSpeedDist < 0) {
            accelerationTime = Math.sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
            fullSpeedDist = 0;
        }

        m_endAccel = accelerationTime - cutoffBegin;
        m_endFullSpeed = m_endAccel + fullSpeedDist / m_constraints.maxVelocity;
        m_endDeccel = m_endFullSpeed + accelerationTime - cutoffEnd;
        State result = new State(m_current.position, m_current.velocity);

        if (dt < m_endAccel) {
            result.velocity += dt * m_constraints.maxAcceleration;
            result.position += (m_current.velocity + dt * m_constraints.maxAcceleration / 2.0) * dt;
            m_finished = false;
        } else if (dt < m_endFullSpeed) {
            result.velocity = m_constraints.maxVelocity;
            result.position += (m_current.velocity + m_endAccel * m_constraints.maxAcceleration / 2.0) * m_endAccel
                    + m_constraints.maxVelocity * (dt - m_endAccel);
            m_finished = false;
        } else if (dt <= m_endDeccel) {
            result.velocity = goal.velocity + (m_endDeccel - dt) * m_constraints.maxAcceleration;
            double timeLeft = m_endDeccel - dt;
            result.position = goal.position
                    - (goal.velocity + timeLeft * m_constraints.maxAcceleration / 2.0) * timeLeft;
            m_finished = false;
        } else {
            result = goal;
            m_finished = true;
        }

        return direct(result);
    }

    /**
     * True if the most recent input to calculate indicates completion.
     */
    public boolean isFinished() {
        return m_finished;
    }

    /**
     * Returns true if the profile inverted.
     *
     * <p>
     * The profile is inverted if goal position is less than the initial position.
     *
     * @param initial The initial state (usually the current state).
     * @param goal    The desired state when the profile is complete.
     */
    private static boolean shouldFlipAcceleration(State initial, State goal) {
        return initial.position > goal.position;
    }

    /**
     * Flip the sign of the velocity and position if the profile is inverted, i.e.
     * moving to the left.
     */
    private State direct(State in) {
        State result = new State(in.position, in.velocity);
        result.position = result.position * m_direction;
        result.velocity = result.velocity * m_direction;
        return result;
    }
}
