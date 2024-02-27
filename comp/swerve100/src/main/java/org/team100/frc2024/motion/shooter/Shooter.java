package org.team100.frc2024.motion.shooter;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.simple.Speeding;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase implements Speeding {
    public abstract void forward();
    public abstract void stop();
    public abstract boolean readyToShoot(SwerveDriveSubsystem m_drive);
    public abstract void pivotAndRamp(SwerveDriveSubsystem m_drive, double kThreshold);
    public abstract void setAngleWithOverride(Double angle, double pivotUp, double pivotDown);
    public abstract void setAngle(Double angle);

    public abstract double getAngle();
    public abstract double getVelocity();
    public abstract void setDutyCycle(double value);
    public abstract double getPivotPosition();
    public abstract void setPivotPosition(double value);
    public abstract void reset();
    public abstract void rezero();

    public abstract void feed();








}
