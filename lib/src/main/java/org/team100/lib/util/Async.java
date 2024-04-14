package org.team100.lib.util;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.io.Writer;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

/**
 * Runs low-priority, timing-insensitive stuff asynchronously.
 * 
 * Instead of a WPILib Notifier, this uses a java executor, in order to set the
 * thread priority really low and avoid the notifier interrupt.
 */
public class Async {
    public static final Async runner = new Async();

    private final ScheduledExecutorService m_scheduler;

    /** Run in t sec and every t sec thereafter. */
    public void addPeriodic(Runnable runnable, double periodS) {
        long periodMS = (long) (periodS * 1000);
        m_scheduler.scheduleAtFixedRate(
                new CrashWrapper(runnable), periodMS, periodMS, TimeUnit.MILLISECONDS);
    }

    private Async() {
        m_scheduler = Executors.newSingleThreadScheduledExecutor(
                new MinPriorityThreads());
    }

    private static class MinPriorityThreads implements ThreadFactory {
        @Override
        public Thread newThread(Runnable r) {
            Thread thread = new Thread(r);
            thread.setPriority(1);
            return thread;
        }
    }

    private static class CrashWrapper implements Runnable {
        private final Runnable m_runnable;

        private CrashWrapper(Runnable runnable) {
            m_runnable = runnable;
        }

        @Override
        public void run() {
            try {
                m_runnable.run();
            } catch (Throwable e) {
                Util.warn(e.toString());
                Writer writer = new StringWriter();
                e.printStackTrace(new PrintWriter(writer));
                Util.warn(writer.toString());
            }
        }
    }
}