package org.team100.lib.util;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.io.Writer;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * Runs low-priority stuff asynchronously in a single notifier. It's similar to
 * the WPILib mechanism in the main loop.
 */
public class Async {
    public static final Async runner = new Async();

    private final PriorityQueue<Callback> m_callbacks;
    private final Notifier m_notifier;

    private Async() {
        m_callbacks = new PriorityQueue<>();
        m_notifier = new Notifier(this::run);
        m_notifier.setName("Async Runner");
        m_notifier.startPeriodic(0.1);
    }

    private void run() {
        double curTimeSec = Timer.getFPGATimestamp();
        while (m_callbacks.peek().m_expirationTimeS <= curTimeSec) {
            Callback callback = m_callbacks.poll();
            try {
                callback.m_runnable.run();
            } catch (Throwable e) {
                // no need to bother the notifier.
                Util.warn(e.toString());
                Writer writer = new StringWriter();
                e.printStackTrace(new PrintWriter(writer));
                Util.warn(writer.toString());
            }
            callback.m_expirationTimeS += callback.m_periodS;
            m_callbacks.add(callback);
        }
    }

    /** Run in t sec and every t sec thereafter. */
    public void addPeriodic(Runnable runnable, double periodS) {
        m_callbacks.add(new Callback(runnable, periodS));
    }

    /** This is cribbed from TimedRobot.java */
    private static class Callback implements Comparable<Callback> {
        private final Runnable m_runnable;
        private final double m_periodS;

        private double m_expirationTimeS;

        private Callback(Runnable runnable, double periodS) {
            m_runnable = runnable;
            m_periodS = periodS;
            m_expirationTimeS = Timer.getFPGATimestamp() + periodS;
        }

        @Override
        public boolean equals(Object rhs) {
            if (rhs instanceof Callback) {
                return Double.compare(m_expirationTimeS, ((Callback) rhs).m_expirationTimeS) == 0;
            }
            return false;
        }

        @Override
        public int hashCode() {
            return Double.hashCode(m_expirationTimeS);
        }

        @Override
        public int compareTo(Callback rhs) {
            // Elements with sooner expiration times are sorted as lesser. The head of
            // Java's PriorityQueue is the least element.
            return Double.compare(m_expirationTimeS, rhs.m_expirationTimeS);
        }
    }
}