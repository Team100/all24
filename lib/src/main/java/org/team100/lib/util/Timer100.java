package org.team100.lib.util;

import edu.wpi.first.wpilibj.RobotController;

/**
 * This exists because the WPI Timer class does not provide access to the
 * "running" field, which is just beyond.
 */
public class Timer100 {
    private double m_startTime;
    private double m_accumulatedTime;
    private boolean m_running;

    public Timer100() {
        reset();
    }

    public double get() {
        if (m_running) {
            return m_accumulatedTime + (getMsClock() - m_startTime) / 1000.0;
        } else {
            return m_accumulatedTime;
        }
    }

    public boolean isRunning() {
        return m_running;
    }

    public boolean isStopped() {
        return !m_running;
    }

    public final void reset() {
        m_accumulatedTime = 0;
        m_startTime = getMsClock();
    }

    public void start() {
        if (!m_running) {
            m_startTime = getMsClock();
            m_running = true;
        }
    }

    public void restart() {
        if (m_running) {
            stop();
        }
        reset();
        start();
    }

    public void stop() {
        m_accumulatedTime = get();
        m_running = false;
    }

    public boolean hasElapsed(double seconds) {
        return get() >= seconds;
    }

    public boolean advanceIfElapsed(double seconds) {
        if (get() >= seconds) {
            // Advance the start time by the period.
            // Don't set it to the current time... we want to avoid drift.
            m_startTime += seconds * 1000;
            return true;
        } else {
            return false;
        }
    }

    //////////////////////////////////

    private double getMsClock() {
        return RobotController.getFPGATime() / 1000.0;
    }
}
