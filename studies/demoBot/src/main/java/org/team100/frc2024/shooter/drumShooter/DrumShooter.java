package org.team100.frc2024.shooter.drumShooter;

import java.util.OptionalDouble;

import org.ejml.simple.UnsupportedOperation;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

/**
 * Direct-drive shooter with left and right drums.
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.1m => 0.314 m/turn
 * therefore top speed is around 30 m/s.
 * 
 * Empirically it seems to take a second or so to spin
 * up, so set the acceleration a bit higher than that to start.
 */
public class DrumShooter implements Glassy {

    private final SupplierLogger m_logger;

    private final LinearVelocityServo m_leftRoller;
    private final LinearVelocityServo m_rightRoller;

    private double currentDesiredLeftVelocity = 0;
    private double currentDesiredRightVelocity = 0;

    public DrumShooter(
            SupplierLogger parent,
            LinearVelocityServo leftRoller,
            LinearVelocityServo rightRoller) {
        m_logger = parent.child(this);
        m_leftRoller = leftRoller;
        m_rightRoller = rightRoller;
    }

    public void set(double velocityM_S) {
        m_leftRoller.setVelocityM_S(velocityM_S);
        m_rightRoller.setVelocityM_S(velocityM_S);
        currentDesiredLeftVelocity = velocityM_S;
        currentDesiredRightVelocity = velocityM_S;
        m_logger.logDouble(Level.TRACE, "Left Shooter Desired",()-> velocityM_S);
        m_logger.logDouble(Level.TRACE, "Right Shooter Desired",()-> velocityM_S);
    }

    public void setIndividual(double leftVelocityM_S, double rightVelocityM_S) {
        m_leftRoller.setVelocityM_S(leftVelocityM_S);
        m_rightRoller.setVelocityM_S(rightVelocityM_S);
        currentDesiredLeftVelocity = leftVelocityM_S;
        currentDesiredRightVelocity = rightVelocityM_S;
        m_logger.logDouble(Level.TRACE, "Left Shooter Desired",()-> rightVelocityM_S);
        m_logger.logDouble(Level.TRACE, "Right Shooter Desired",()-> leftVelocityM_S);
    }

    /** Returns the average of the two rollers */
    public double getVelocity() {
        return getLeftVelocityM_S() + getRightVelocityM_S() / 2;
    }

    public boolean atVeloctity() {
        return atVeloctity(0.5);
    }

    /**
     * 
     * @param tolerance Units are M_S
     * @return If the absolute value of the error is less than the tolerance
     */
    public boolean atVeloctity(double tolerance) {
        return Math.abs(rightError()) < tolerance && Math.abs(leftError()) < tolerance;
    }

    public double rightError() {
        return getRightVelocityM_S() - currentDesiredRightVelocity;
    }

    public double leftError() {
        return getLeftVelocityM_S() - currentDesiredLeftVelocity;
    }

    public double getLeftVelocityM_S() {
        OptionalDouble velocity = m_leftRoller.getVelocity();
        if (velocity.isEmpty()) {
            throw new UnsupportedOperation("Left shooter roller sensor is broken!");
        }
        return velocity.getAsDouble();
    }

    public double getRightVelocityM_S() {
        OptionalDouble velocity = m_rightRoller.getVelocity();
        if (velocity.isEmpty()) {
            throw new UnsupportedOperation("Right shooter roller sensor is broken!");
        }
        return velocity.getAsDouble();
    }
    
    @Override
    public String getGlassName() {
        return "DrumShooter";
    }
}