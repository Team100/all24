package org.team100.lib.profile;

import org.team100.lib.controller.State100;

/**
 * Wraps a profile and limits the jerk produced. Note that this may cause the
 * profile to overshoot the desired output state.
 */
public class JerkLimiter implements Profile100 {
    private final Profile100 m_delegate;
    private final double m_maxJerk;

    public JerkLimiter(Profile100 delegate, double maxJerk) {
        m_delegate = delegate;
        m_maxJerk = maxJerk;
    }

    /**
     * If the delegate stays within the jerk limit, then use the delegate's output.
     * 
     * If not, create a new max-jerk output in the desired direction.
     * 
     * @param dt time step
     * @param i  initial state
     * @param g  goal state
     */
    @Override
    public State100 calculate(double dt, State100 i, State100 g) {
        State100 guess = m_delegate.calculate(dt, i, g);
        double delegateJerk = (guess.a() - i.a()) / dt;
        if (Math.abs(delegateJerk) < m_maxJerk)
            return guess;
        double j = m_maxJerk * Math.signum(delegateJerk);
        double a = i.a() + j * dt;
        double v = i.v() + i.a() * dt + j * Math.pow(dt, 2) / 2;
        double x = i.x() + i.v() * dt + i.a() * Math.pow(dt, 2) / 2 + j * Math.pow(dt, 3) / 6;
        return new State100(x, v, a);
    }

}
