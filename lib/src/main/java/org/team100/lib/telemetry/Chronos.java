package org.team100.lib.telemetry;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.wpilibj.Timer;

/**
 * Keeps track of how long things take.
 * The general idea is to reset all the timers periodically, to let them
 * accumulate time, and then to spit out the fraction of time spent in each one.
 */
public class Chronos {
    private static final Chronos instance = new Chronos();

    /** This is just for tasks that are running. */
    private final Map<String, Double> m_starts = new ConcurrentHashMap<>();

    /** This accumulates between calls to log. */
    private final Map<String, Double> m_durations = new ConcurrentHashMap<>();

    private double m_time = Timer.getFPGATimestamp();

    public static Chronos get() {
        return instance;
    }

    /**
     * Don't leave these hanging, use try/finally. Will be confused by parallel
     * tasks.
     */
    public void start(String name) {
        m_starts.put(name, Timer.getFPGATimestamp());
    }

    /** Expected to be inside a finally block. */
    public void end(String name) {
        double start = m_starts.get(name);
        double duration = Timer.getFPGATimestamp() - start;
        double prevDuration = m_durations.getOrDefault(name, 0.0);
        m_durations.put(name, duration + prevDuration);
    }

    public Map<String, Double> durations() {
        return m_durations;
    }

    public double elapsed() {
        return Timer.getFPGATimestamp() - m_time;
    }

    public void reset() {
        m_time = Timer.getFPGATimestamp();
        m_durations.clear();
    }

}
