package org.team100.lib.commands;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.io.Writer;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Executes the command in a separate thread, using a java executor.
 * Calculates dt.
 * 
 * The glass name leaf is always the implementing class name.
 */
public abstract class Command100 extends Command implements Glassy {
    // TODO: allow subclasses to specify the period they want.
    private static final int kExecutePeriodMilliS = 5;

    private static final Telemetry t = Telemetry.get();

    private static final ScheduledExecutorService m_scheduler = Executors.newSingleThreadScheduledExecutor(
            new MaxPriorityThreads());

    protected final String m_name;
    private double prevTime;
    private Future<?> m_task;

    protected Command100() {
        m_name = Names.append(getGlassName(), this);
    }

    public void initialize100() {
        //
    }

    /**
     * @param dt duration since the previous call.
     */
    public abstract void execute100(double dt);

    public void end100(boolean interrupted) {
        //
    }

    @Override
    public final void initialize() {
        t.log(Level.DEBUG, m_name, "command state", "initialize");
        prevTime = Timer.getFPGATimestamp();
        initialize100();
        if (m_scheduler.isShutdown()) {
            Util.warn("TEST MODE: skipping scheduler.");
            return;
        }
        m_task = m_scheduler.scheduleAtFixedRate(new CrashWrapper(), 0, kExecutePeriodMilliS, TimeUnit.MILLISECONDS);
    }

    @Override
    public final void execute() {
        // command100 execute does nothing, the scheduler does the work.
        if (m_scheduler.isShutdown()) {
            throw new IllegalStateException(
                    "Tests should not call Command.execute(), use Command100.execute100() instead.");
        }
    }

    @Override
    public final void end(boolean interrupted) {
        // "false" -> don't interrupt the thread
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
                t.log(Level.DEBUG, m_name, "command state", "execute");
                double now = Timer.getFPGATimestamp();
                double dt = now - prevTime;
                t.log(Level.DEBUG, m_name, "dt", dt);
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
