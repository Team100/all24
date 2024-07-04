package org.team100.lib.commands;

import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.io.StringWriter;
import java.io.PrintWriter;
import java.io.Writer;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Calculates dt.
 * 
 * The glass name leaf is always the implementing class name.
 */
public abstract class Command100 extends Command implements Glassy {
    private static final int kExecutePeriodMilliS = 5;

    private static final ScheduledExecutorService m_scheduler = Executors.newSingleThreadScheduledExecutor(
            new MaxPriorityThreads());

    protected final Logger m_logger;
    protected final String m_name;

    private double prevTime;
    private Future<?> m_task;

    protected Command100(Logger parent) {
        m_name = Names.append(Command100.class.getSimpleName(), this);
        m_logger = parent.child(this);
    }

    public void initialize100() {
        //
    }

    /** @param dt duration since the previous call. */
    public abstract void execute100(double dt);

    public void end100(boolean interrupted) {
        //
    }

    @Override
    public final void initialize() {
        m_logger.log(Level.DEBUG, "command state", "initialize");
        prevTime = Timer.getFPGATimestamp();
        initialize100();
        if (Experiments.instance.enabled(Experiment.UseCommandExecutor)) {
            if (m_scheduler.isShutdown()) {
                Util.warn("TEST MODE: skipping scheduler.");
                return;
            }
            m_task = m_scheduler.scheduleAtFixedRate(new CrashWrapper(), 0, kExecutePeriodMilliS,
                    TimeUnit.MILLISECONDS);
        }
    }

    @Override
    public final void execute() {
        if (Experiments.instance.enabled(Experiment.UseCommandExecutor)) {
            return;
        }
        m_logger.log(Level.DEBUG, "command state", "execute");
        double now = Timer.getFPGATimestamp();
        double dt = now - prevTime;
        m_logger.logDouble(Level.DEBUG, "dt", ()->dt);
        prevTime = now;
        execute100(dt);
    }

    @Override
    public void end(boolean interrupted) {
        if (m_task != null)
            m_task.cancel(false);
        end100(interrupted);
    }

    @Override
    public final String getGlassName() {
        return this.getClass().getSimpleName();
    }

    /**
     * {@link #execute100()} is usually run in a separate thread, but for tests,
     * it's better to run it in the test itself.
     */
    public static void shutDownForTest() {
        m_scheduler.shutdownNow();
    }

    private static class MaxPriorityThreads implements ThreadFactory {
        private final AtomicInteger id;

        private MaxPriorityThreads() {
            id = new AtomicInteger();
        }

        /**
         * Should only ever be called once but just in case, there's an incrementing
         * id so we can tell what's happening.
         */
        @Override
        public Thread newThread(Runnable r) {
            Thread thread = new Thread(r);
            thread.setPriority(10);
            thread.setDaemon(true);
            thread.setName("Command100 Thread " + id.getAndIncrement());
            return thread;
        }
    }

    private class CrashWrapper implements Runnable {

        @Override
        public void run() {
            try {
                m_logger.log(Level.DEBUG, "command state", "execute");
                double now = Timer.getFPGATimestamp();
                double dt = now - prevTime;
                m_logger.logDouble(Level.DEBUG, "dt",()-> dt);
                prevTime = now;
                execute100(dt);
            } catch (Throwable e) {
                Util.warn(e.toString());
                Writer writer = new StringWriter();
                e.printStackTrace(new PrintWriter(writer));
                Util.warn(writer.toString());
            }
        }
    }

}