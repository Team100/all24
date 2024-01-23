package org.team100.frc2024.motion.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: add shooter to selftest.
 */
public abstract class Shooter extends SubsystemBase {
    public abstract void forward();
    public abstract void stop();
    public abstract boolean readyToShoot();
}
