package org.team100.lib.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Calculates dt.
 */
public abstract class Command100 extends Command {

    private double prevTime;

    public abstract void initialize100();
    
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
