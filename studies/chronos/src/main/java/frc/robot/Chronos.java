package org.team100.lib.telemetry;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.Timer;

/**
 * Keeps track of how long things take.
 * The general idea is to reset all the timers periodically, to let them
 * accumulate time, and then to spit out the time spent in each one.
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

    private final Map<String, Double> m_durationsS = new ConcurrentHashMap<>();
    
    private final SupplierLogger2 m_logger;
    private final DoubleSupplierLogger2 m_log_elapsed;

    private double m_time;

    public Chronos(SupplierLogger2 logger) {
        m_logger = logger;
        m_log_elapsed = m_logger.doubleLogger(Level.COMP, "elapsed (s)");

        m_time = Timer.getFPGATimestamp();
    }

    /** Call end() on this sample when done. */
    public Sample sample(String name) {
        return new Sample(name);
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

    public void dump() {
        final double elapsed = elapsed();
        m_log_elapsed.log(() -> elapsed);
        for (Map.Entry<String, Double> durations : m_durationsS.entrySet()) {
            Double duration = durations.getValue();
            String name = durations.getKey();
            m_logger.doubleLogger(Level.COMP, "duration (s)/" + name).log(() -> duration);
            double fraction = duration / elapsed;
            m_logger.doubleLogger(Level.COMP, "fraction (pct)/" + name).log(() -> 100.0 * fraction);
        }
        reset();

    }

}
