package org.team100.frc2024.motion.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: add shooter to selftest.
 */
public abstract class Intake extends SubsystemBase {
    public abstract void intake();
    public abstract void outtake();
    public abstract void stop();
}
