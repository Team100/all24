package org.team100.frc2024.shooter.drumShooter;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.util.Neo550Factory;

public class ShooterCollection {
    private static final String kShooter = "Shooter Wheels";
    private static final String kLeft = "Left";
    private static final String kRight = "Right";
    private static final double m_gearRatio = 5.2307692308;

    private final LinearVelocityServo m_leftShooter;
    private final LinearVelocityServo m_rightShooter;

    private ShooterCollection(LinearVelocityServo leftShooter, LinearVelocityServo rightShooter) {
        m_leftShooter = leftShooter;
        m_rightShooter = rightShooter;
    }

    public static ShooterCollection get(
        LoggerFactory parent,
        int currentLimit) {
            LoggerFactory collectionLogger = parent.child(kShooter);
            double shooterWheelDiameterM = .33;
            switch (Identity.instance) {
                case DEMO_BOT:
                //TODO get the real diameter, gearRatios, and canIDs, and Indexer accel
                LinearVelocityServo leftDrum = Neo550Factory.getNEO550VelocityServo(kLeft, collectionLogger,currentLimit,39,m_gearRatio,MotorPhase.FORWARD, shooterWheelDiameterM);
                LinearVelocityServo rightDrum = Neo550Factory.getNEO550VelocityServo(kRight, collectionLogger,currentLimit,19,m_gearRatio,MotorPhase.REVERSE, shooterWheelDiameterM);
                return new ShooterCollection(leftDrum, rightDrum);
                case BLANK:
                default:
                    return new ShooterCollection(
                        Neo550Factory.simulatedDriveServo(collectionLogger.child(kLeft)),
                        Neo550Factory.simulatedDriveServo(collectionLogger.child(kRight)));
                    }
    }

    public LinearVelocityServo getLeftShooter() {
        return m_leftShooter;
    }

    public LinearVelocityServo getRightShooter() {
        return m_rightShooter;
    }
}
