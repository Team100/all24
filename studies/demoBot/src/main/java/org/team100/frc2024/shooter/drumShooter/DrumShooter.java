package org.team100.frc2024.shooter.drumShooter;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

/**
 * Direct-drive shooter with top and bottom drums.
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
    public OptionalDouble getVelocity() {
        OptionalDouble leftVelocity = getLeftVelocity();
        OptionalDouble rightVelocity = getRightVelocity();
        if (leftVelocity.isEmpty() || rightVelocity.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of((leftVelocity.getAsDouble() + rightVelocity.getAsDouble()) / 2);
    }

    public boolean atVeloctity() {
        return Math.abs(getRightVelocity().getAsDouble() - currentDesiredRightVelocity) < 0.5 && Math.abs(getLeftVelocity().getAsDouble() - currentDesiredLeftVelocity) < 0.5;
    }

    public OptionalDouble getLeftVelocity() {
        return m_leftRoller.getVelocity();
    }

    public OptionalDouble getRightVelocity() {
        return m_rightRoller.getVelocity();
    }
    
    @Override
    public String getGlassName() {
        return "DrumShooter";
    }
}