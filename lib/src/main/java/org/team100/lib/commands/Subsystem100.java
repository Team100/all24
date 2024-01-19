package org.team100.lib.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** provides periodic with dt. */
public abstract class Subsystem100 extends SubsystemBase {

    private double prevTime = Timer.getFPGATimestamp();

    public abstract void periodic100(double dt);

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - prevTime;
        prevTime = now;
        periodic100(dt);
    }

}
