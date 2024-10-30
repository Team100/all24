package org.team100.frc2024.shooter.drumShooter;

import java.util.OptionalDouble;

import org.ejml.simple.UnsupportedOperation;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.servo.LinearVelocityServo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
public class DrumShooter extends SubsystemBase implements Glassy {

    private final DoubleLogger m_leftlogger;
    private final DoubleLogger m_rightlogger;
    private final BooleanLogger m_atVelocitylogger;

    private final LinearVelocityServo m_leftRoller;
    private final LinearVelocityServo m_rightRoller;
    private final double shooterVelocityM_S = 10;

    private double currentDesiredLeftVelocity = 0;
    private double currentDesiredRightVelocity = 0;
    
    private boolean atVelocity;

    public DrumShooter(
            LoggerFactory parent,
            ShooterCollection shooterCollection) {
        LoggerFactory logger = parent.child(this);
        m_atVelocitylogger = logger.booleanLogger(Level.TRACE, "At velocity");
        m_leftlogger = logger.doubleLogger(Level.TRACE, "Left Shooter Desired");
        m_rightlogger = logger.doubleLogger(Level.TRACE, "Right Shooter Desired");
        m_leftRoller = shooterCollection.getLeftShooter();
        m_rightRoller = shooterCollection.getRightShooter();
    }

    public void set(double velocityM_S) {
        m_leftRoller.setVelocityM_S(velocityM_S);
        m_rightRoller.setVelocityM_S(velocityM_S);
        currentDesiredLeftVelocity = velocityM_S;
        currentDesiredRightVelocity = velocityM_S;
        m_leftlogger.log(()-> velocityM_S);
        m_rightlogger.log(()-> velocityM_S);
    }

    public void stop() {
        set(0);
    }

    public void setIndividual(double leftVelocityM_S, double rightVelocityM_S) {
        m_leftRoller.setVelocityM_S(leftVelocityM_S);
        m_rightRoller.setVelocityM_S(rightVelocityM_S);
        currentDesiredLeftVelocity = leftVelocityM_S;
        currentDesiredRightVelocity = rightVelocityM_S;
        m_leftlogger.log(()-> rightVelocityM_S);
        m_rightlogger.log(()-> leftVelocityM_S);
    }

    public void spinUp() {
        set(shooterVelocityM_S);
    }

    /** Returns the average of the two rollers */
    public double getVelocity() {
        return getLeftVelocityM_S() + getRightVelocityM_S() / 2;
    }

    public boolean atVeloctity() {
        return atVeloctity(10);
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
    public void periodic() {
        atVelocity = atVeloctity();
        m_atVelocitylogger.log(() -> atVelocity);
    }

    @Override
    public String getGlassName() {
        return "DrumShooter";
    }
}