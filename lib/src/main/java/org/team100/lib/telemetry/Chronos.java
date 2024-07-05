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
    /**
     * Use this as follows:
     * 
     * <pre>
     * Sample s = Chronos.get().sample("foo");
     * try {
     *     ... code under test ...
     * } finally {
     *     s.end();
     * }
     * </pre>
     */
    public class Sample {
        private final String m_name;
        private final double m_startS;

        public Sample(String name) {
            m_name = name;
            m_startS = Timer.getFPGATimestamp();
        }

        public void end() {
            double durationS = Timer.getFPGATimestamp() - m_startS;
            m_durationsS.merge(m_name, durationS, Double::sum);
        }
    }

    private static final Chronos instance = new Chronos();

    private final Map<String, Double> m_durationsS = new ConcurrentHashMap<>();

    private double m_time = Timer.getFPGATimestamp();

    public static Chronos get() {
        return instance;
    }

    /** Call end() on this sample when done. */
    public Sample sample(String name) {
        return new Sample(name);
    }

    public Map<String, Double> durations() {
        return m_durationsS;
    }

    public double elapsed() {
        return Timer.getFPGATimestamp() - m_time;
    }

    public void reset() {
        m_time = Timer.getFPGATimestamp();
        for (String key : m_durationsS.keySet()) {
            m_durationsS.put(key, 0.0);
        }
    }

}
