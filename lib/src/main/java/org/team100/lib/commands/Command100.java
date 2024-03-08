package org.team100.lib.commands;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Calculates dt.
 * 
 * The glass name leaf is always the implementing class name.
 */
public abstract class Command100 extends Command implements Glassy {
    private static final Telemetry t = Telemetry.get();

    protected final String m_name;

    private double prevTime;

    protected Command100() {
        m_name = Names.append(Command100.class.getSimpleName(), this);
    }

    public void initialize100() {
        //
    }

    /** @param dt duration since the previous call. */
    public abstract void execute100(double dt);

    @Override
    public final void initialize() {
        t.log(Level.DEBUG, m_name, "command state", "initialize");
        prevTime = Timer.getFPGATimestamp();
        initialize100();
    }

    @Override
    public final void execute() {
        t.log(Level.DEBUG, m_name, "command state", "execute");
        double now = Timer.getFPGATimestamp();
        double dt = now - prevTime;
        prevTime = now;
        execute100(dt);
    }

    @Override
    public final String getGlassName() {
        return this.getClass().getSimpleName();
    }

    
}
