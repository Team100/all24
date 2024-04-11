package org.team100.frc2024.motion.shooter;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.simple.Speeding;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase implements Speeding, Glassy {
    public abstract void forward();
    public abstract void stop();
    public abstract boolean readyToShoot(Alliance alliance, SwerveDriveSubsystem m_drive);
    public abstract void pivotAndRamp(SwerveDriveSubsystem m_drive, double kThreshold);
    public abstract void setAngleWithOverride(Double angle, double pivotUp, double pivotDown);
    public abstract void setAngle(Double angle);

    public abstract double getAngleRad();
    public abstract boolean atVelocitySetpoint();
    public abstract boolean atVelocitySetpoint(boolean bool);

    public abstract void setDutyCycle(double value);
    public abstract double getPivotPosition();
    public abstract void setPivotPosition(double value);
    public abstract void outtake();

    public abstract void reset();
    public abstract void rezero();

    public abstract void feed();

    @Override
    public String getGlassName() {
        return "Shooter";
    }

}
