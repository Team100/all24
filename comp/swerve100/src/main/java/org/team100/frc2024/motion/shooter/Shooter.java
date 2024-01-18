package org.team100.frc2024.motion.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    public abstract void setVelocity(double value);    
}
