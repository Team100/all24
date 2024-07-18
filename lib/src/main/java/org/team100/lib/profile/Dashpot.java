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

    public Dashpot(Profile100 fast, Profile100 slow, double distance) {
        m_fast = fast;
        m_slow = slow;
        m_distance = distance;
    }

    @Override
    public State100 calculate(double dt, State100 initial, State100 goal) {
        if (Math.abs(goal.x() - initial.x()) > m_distance)
            return m_fast.calculate(dt, initial, goal);
        return m_slow.calculate(dt, initial, goal);
    }

}
