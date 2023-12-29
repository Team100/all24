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

    /**
     * True if the most-recent input to calculate indicates completion.
     */
    private boolean m_finished;

    public TrapezoidProfile100(Constraints constraints) {
        m_constraints = constraints;
    }

    /**
     * Given current and goal states, calculate the state at dt in the future,
     * within the constraints.
     * 
     * The current state is usually not a measurement, to avoid adding noise and
     * delay; instead, use the previous output of this method.
     *
     * @param dt         Time in the future we're aiming for
     * @param in_goal    Desired end state
     * @param in_current Current state
     */
    public State calculate(double dt, final State in_goal, final State in_current) {
        State goal = new State(in_goal);
        State current = new State(in_current);
        // The logic below assumes rightward profile; for leftward, invert.
        boolean inverted = current.position > goal.position;
        if (inverted) {
            current.position *= -1;
            current.velocity *= -1;
            goal.position *= -1;
            goal.velocity *= -1;
        }

        if (current.velocity > m_constraints.maxVelocity) {
            current.velocity = m_constraints.maxVelocity;
        }

        // Deal with a possibly truncated motion profile (with nonzero initial or
        // final velocity) by calculating the parameters as if the profile began and
        // ended at zero velocity
        double cutoffBegin = current.velocity / m_constraints.maxAcceleration;
        double cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

        double cutoffEnd = goal.velocity / m_constraints.maxAcceleration;
        double cutoffDistEnd = cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one

        double fullTrapezoidDist = cutoffDistBegin + (goal.position - current.position) + cutoffDistEnd;
        double accelerationTime = m_constraints.maxVelocity / m_constraints.maxAcceleration;

        double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxAcceleration;

        // Handle the case where the profile never reaches full speed
        if (fullSpeedDist < 0) {
            accelerationTime = Math.sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
            fullSpeedDist = 0;
        }

        // Time of the end of the acceleration period.
        double endAccelS = accelerationTime - cutoffBegin;

        // Time of the end of the cruise period. Same as endAccel for triangular
        // profiles.
        double endCruiseS = endAccelS + fullSpeedDist / m_constraints.maxVelocity;

        // Time of the end of the deceleration period; also the end of the profile.
        double endDecelS = endCruiseS + accelerationTime - cutoffEnd;

        State result = new State(current.position, current.velocity);

        if (dt < endAccelS) {
            result.velocity += dt * m_constraints.maxAcceleration;
            result.position += (current.velocity + dt * m_constraints.maxAcceleration / 2.0) * dt;
            m_finished = false;
        } else if (dt < endCruiseS) {
            result.velocity = m_constraints.maxVelocity;
            result.position += (current.velocity + endAccelS * m_constraints.maxAcceleration / 2.0) * endAccelS
                    + m_constraints.maxVelocity * (dt - endAccelS);
            m_finished = false;
        } else if (dt <= endDecelS) {
            result.velocity = goal.velocity + (endDecelS - dt) * m_constraints.maxAcceleration;
            double timeLeft = endDecelS - dt;
            result.position = goal.position
                    - (goal.velocity + timeLeft * m_constraints.maxAcceleration / 2.0) * timeLeft;
            m_finished = false;
        } else {
            result = goal;
            m_finished = true;
        }
        if (inverted) {
            result.position *= -1;
            result.velocity *= -1;
        }
        return result;
    }

    /**
     * True if the most recent input to calculate indicates completion.
     */
    public boolean isFinished() {
        return m_finished;
    }
}
