package org.team100.lib.async;

import org.team100.lib.framework.TimedRobot100;

/**
 * Support async periodic execution using TimedRobot callbacks.
 * 
 * This involves no new threads, just more stuff on the main thread.
 */
public class TimedRobotAsync implements Async {
    private final TimedRobot100 m_robot;

    TimedRobotAsync(TimedRobot100 robot) {
        m_robot = robot;
    }

    @Override
    public void addPeriodic(Runnable runnable, double periodS, String name) {
        m_robot.addPeriodic(runnable, periodS, name);
    }

}
