package org.team100.lib.profile;

import org.team100.lib.controller.State100;

/**
 * Contains two profiles, switches between them depending on the distance to the
 * goal.
 * 
 * The idea is to use a high-effort profile when far from the goal, and then
 * switch to a low-effort profile when close, like the soft-close dashpot in a
 * drawer slide.
 */
public class Dashpot implements Profile100 {
    private final Profile100 m_fast;
    private final Profile100 m_slow;
    private final double m_distance;
    private final double m_velocity;

    public Dashpot(Profile100 fast, Profile100 slow, double distance, double velocity) {
        m_fast = fast;
        m_slow = slow;
        m_distance = distance;
        m_velocity = velocity;
    }

    @Override
    public State100 calculate(double dt, State100 initial, State100 goal) {
        double togo = goal.x() - initial.x();
        if (Math.abs(togo) > m_distance) {
            // adjust the goal X to give the slow profile room to operate
            double adjustedX = goal.x() - Math.signum(togo) * m_distance;
            // the slow profile wants to be cruising at the handoff point.
            double adjustedV = Math.signum(togo) * m_velocity;
            State100 adjustedGoal = new State100(
                    adjustedX,
                    adjustedV,
                    goal.a());
            return m_fast.calculate(dt, initial, adjustedGoal);
        }
        return m_slow.calculate(dt, initial, goal);
    }

}
