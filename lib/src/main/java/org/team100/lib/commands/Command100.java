package org.team100.lib.commands;

import java.util.concurrent.Future;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.StringSupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Calculates dt.
 * 
 * The glass name leaf is always the implementing class name.
 */
public abstract class Command100 extends Command implements Glassy {
    protected final SupplierLogger2 m_logger;
    private final StringSupplierLogger2 m_log_command_state;
    private final DoubleSupplierLogger2 m_log_dt;

    private double prevTime;
    private Future<?> m_task;

    protected Command100(SupplierLogger2 parent) {
        m_logger = parent.child(this);
        m_log_command_state = m_logger.stringLogger(Level.TRACE, "command state");
        m_log_dt = m_logger.doubleLogger(Level.TRACE, "dt");
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
        m_log_command_state.log(() -> "initialize");
        prevTime = Timer.getFPGATimestamp();
        initialize100();
    }

    @Override
    public final void execute() {
        m_log_command_state.log(() -> "execute");
        double now = Timer.getFPGATimestamp();
        double dt = now - prevTime;
        m_log_dt.log(() -> dt);
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
}