package org.team100.frc2024.motion.intake;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.simple.Speeding;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase implements Speeding, Glassy  {
    public abstract void intake();
    public abstract void intakeSmart();

    public abstract void outtake();
    public abstract void stop();
    public abstract void runUpper();

    @Override
    public String getGlassName() {
        return "Intake";
    }
}
