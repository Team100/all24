package org.team100.frc2024.shooter.shooter;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.motion.SimpleLinearMechanism;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motor.BareMotor;
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

    private final LinearVelocityServo leftRoller;
    private final LinearVelocityServo rightRoller;

    private double currentDesiredLeftVelocity = 0;
    private double currentDesiredRightVelocity = 0;

    public DrumShooter(
            SupplierLogger parent,
            BareMotor rightMotor,
            BareMotor leftMotor,
            IncrementalBareEncoder rightEncoder, //5
            IncrementalBareEncoder leftEncoder, //4
            double kDriveReduction,
            double kWheelDiameterM) {
        m_logger = parent.child(this);
        SupplierLogger leftLogger = m_logger.child("Left");
        SupplierLogger rightLogger = m_logger.child("Right");

        LinearMechanism leftMech = new SimpleLinearMechanism(
                leftMotor,
                leftEncoder,
                kDriveReduction,
                kWheelDiameterM);
        leftRoller = new OutboardLinearVelocityServo(leftLogger, leftMech);

        LinearMechanism rightMech = new SimpleLinearMechanism(
                rightMotor,
                rightEncoder,
                kDriveReduction,
                kWheelDiameterM);
        rightRoller = new OutboardLinearVelocityServo(rightLogger, rightMech);
    }

    public void set(double velocityM_S) {
        leftRoller.setVelocityM_S(velocityM_S);
        rightRoller.setVelocityM_S(velocityM_S);
        currentDesiredLeftVelocity = velocityM_S;
        currentDesiredRightVelocity = velocityM_S;
        m_logger.logDouble(Level.TRACE, "Right Shooter Desired",()-> velocityM_S);
        m_logger.logDouble(Level.TRACE, "Left Shooter Desired",()-> velocityM_S);
    }

    public void setIndividual(double leftVelocityM_S, double rightVelocityM_S) {
        leftRoller.setVelocityM_S(leftVelocityM_S);
        rightRoller.setVelocityM_S(rightVelocityM_S);
        currentDesiredLeftVelocity = leftVelocityM_S;
        currentDesiredRightVelocity = rightVelocityM_S;
        m_logger.logDouble(Level.TRACE, "Right Shooter Desired",()-> leftVelocityM_S);
        m_logger.logDouble(Level.TRACE, "Left Shooter Desired",()-> rightVelocityM_S);
    }

    /** Returns the average of the two rollers */
    public OptionalDouble getVelocity() {
        OptionalDouble leftVelocity = leftRoller.getVelocity();
        OptionalDouble rightVelocity = rightRoller.getVelocity();
        if (leftVelocity.isEmpty() || rightVelocity.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of((leftVelocity.getAsDouble() + rightVelocity.getAsDouble()) / 2);
    }

    public boolean atVeloctity() {
        return Math.abs(getRightVelocity().getAsDouble() - currentDesiredRightVelocity) < 0.5 && Math.abs(getLeftVelocity().getAsDouble() - currentDesiredLeftVelocity) < 0.5;
    }

    public OptionalDouble getLeftVelocity() {
        return leftRoller.getVelocity();
    }

    public OptionalDouble getRightVelocity() {
        return rightRoller.getVelocity();
    }
    
    @Override
    public String getGlassName() {
        return "DrumShooter";
    }
}