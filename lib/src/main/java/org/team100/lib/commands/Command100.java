package org.team100.lib.commands;

import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Calculates dt.
 */
public abstract class Command100 extends Command {

    protected final String m_name;

    private double prevTime;

    protected Command100() {
        m_name = Names.append(Names.name(Command100.class), this);
    }

    public void initialize100() {
        //
    }

    /** @param dt duration since the previous call. */
    public abstract void execute100(double dt);

    @Override
    public final void initialize() {
        prevTime = Timer.getFPGATimestamp();
        initialize100();
    }

    @Override
    public final void execute() {
        double now = Timer.getFPGATimestamp();
        double dt = now - prevTime;
        prevTime = now;
        execute100(dt);
    }
}
