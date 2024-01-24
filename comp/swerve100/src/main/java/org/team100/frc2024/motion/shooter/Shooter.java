package org.team100.frc2024.motion.shooter;

import org.team100.lib.motion.simple.Speeding;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase implements Speeding {
    public abstract void forward();
    public abstract void stop();
    public abstract boolean readyToShoot();
}
