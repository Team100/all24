package org.team100.frc2024.motion.shooter;

import org.team100.lib.dashboard.Glassy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase implements Glassy {
    public abstract void forward();
    public abstract void stop();
    public abstract void setAngle(double angleRad);

    public abstract double getAngleRad();
    public abstract boolean atVelocitySetpoint();
    public abstract boolean atVelocitySetpoint(boolean bool);

    public abstract double getPivotPosition();
    public abstract void setPivotPosition(double angleRad);
    public abstract void outtake();
    public abstract double getVelocity();

    public abstract void reset();
    public abstract void rezero();

    public abstract void feed();

    @Override
    public String getGlassName() {
        return "Shooter";
    }

}
