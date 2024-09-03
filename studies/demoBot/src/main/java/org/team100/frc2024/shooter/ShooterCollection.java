package org.team100.frc2024.shooter;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OutboardGravityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.util.Neo550Factory;
import edu.wpi.first.wpilibj.Servo;

public class ShooterCollection {
    private static final String kShooter = "Shooter Wheels";
    private static final String kLeft = "Left";
    private static final String kRight = "Right";
    private static final String kPivot = "Pivot";
    private static final double m_gearRatio = 5.2307692308;
    private static final double m_shooterWheelDiameter = 0.098425;

    private final LinearVelocityServo m_leftShooter;
    private final LinearVelocityServo m_rightShooter;
    private final Servo m_indexer;
    private final OutboardGravityServo m_pivot;

    private ShooterCollection(LinearVelocityServo leftShooter, LinearVelocityServo rightShooter, Servo indexer, OutboardGravityServo pivot) {
        m_leftShooter = leftShooter;
        m_rightShooter = rightShooter;
        m_indexer = indexer;
        m_pivot = pivot;
    }

    public static ShooterCollection get(
        SupplierLogger parent,
        int currentLimit) {
            SupplierLogger collectionLogger = parent.child(kShooter);
            double shooterWheelDiameterM = .33;
            switch (Identity.instance) {
                case DEMO_BOT:
                //TODO get the real diameter, gearRatios, and canIDs, and Indexer accel
                LinearVelocityServo leftDrum = Neo550Factory.getNEO550VelocityServo(kLeft, collectionLogger,currentLimit,4,m_gearRatio,MotorPhase.FORWARD, shooterWheelDiameterM);
                LinearVelocityServo rightDrum = Neo550Factory.getNEO550VelocityServo(kRight, collectionLogger,currentLimit,5,m_gearRatio,MotorPhase.REVERSE, shooterWheelDiameterM);
                Servo indexer = new Servo(1);
                OutboardGravityServo pivot = Neo550Factory.getNEO550GravityServo(kPivot, collectionLogger, currentLimit, 1, m_shooterWheelDiameter, MotorPhase.FORWARD);
                return new ShooterCollection(leftDrum, rightDrum, indexer,pivot);
                case BLANK:
                default:
                    return new ShooterCollection(
                        Neo550Factory.simulatedDriveServo(collectionLogger.child(kLeft)),
                        Neo550Factory.simulatedDriveServo(collectionLogger.child(kRight)),
                        new Servo(1),
                        Neo550Factory.simulatedGravityServo(collectionLogger));
                    }
    }

    public LinearVelocityServo[] getShooters() {
        return new LinearVelocityServo[] {
            m_leftShooter,
            m_rightShooter
        };
    }

    public Servo getIndexer() {
        return m_indexer;
    }

    public OutboardGravityServo getPivot() {
        return m_pivot;
    }
}
