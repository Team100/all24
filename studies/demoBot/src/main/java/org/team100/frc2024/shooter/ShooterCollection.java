package org.team100.frc2024.shooter;

import org.team100.frc2024.shooter.pivot.GravityServo;
import org.team100.lib.config.Identity;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.util.CollectionUtil;

public class ShooterCollection {
    private static final String kShooter = "Shooter Wheels";
    private static final String kLeft = "Left";
    private static final String kRight = "Right";
    private static final String kIndexer = "Indexer";

    private final LinearVelocityServo m_leftShooter;
    private final LinearVelocityServo m_rightShooter;
    private final LinearMechanism m_indexer;
    private final GravityServo m_pivot;
    private final double kIndexWheelDiameterM;
    private final double kMaxIndexAccelM_S2;

    private ShooterCollection(LinearVelocityServo leftShooter, LinearVelocityServo rightShooter, LinearMechanism indexer, double kMaxIndexAccel, double kIndexWheelDiameter) {
        m_leftShooter = leftShooter;
        m_rightShooter = rightShooter;
        m_indexer = indexer;
        m_pivot = null;
        kMaxIndexAccelM_S2 = kMaxIndexAccel;
        kIndexWheelDiameterM = kIndexWheelDiameter;
    }

    public static ShooterCollection get(
        SupplierLogger parent,
        int currentLimit) {
            SupplierLogger collectionLogger = parent.child(kShooter);
            double shooterWheelDiameterM = .33;
            switch (Identity.instance) {
                case CAMERA_DOLLY:
                //TODO get the real diameter, gearRatios, and canIDs, and Indexer accel
                double kIndexWheelDiameterM =.1;
                double kMaxIndexAccel = 80 * Math.PI * kIndexWheelDiameterM;
                LinearVelocityServo leftDrum = CollectionUtil.getNEO550VelocityServo(kLeft, collectionLogger,currentLimit,4,5,MotorPhase.FORWARD, shooterWheelDiameterM);
                LinearVelocityServo rightDrum = CollectionUtil.getNEO550VelocityServo(kRight, collectionLogger,currentLimit,5,5,MotorPhase.REVERSE, shooterWheelDiameterM);
                LinearMechanism indexer = CollectionUtil.getNEO550VelocityMechanism(kIndexer, collectionLogger,currentLimit,10,5,MotorPhase.FORWARD, kIndexWheelDiameterM);
                GravityServo pivot = new GravityServo(null, collectionLogger, null, null, kMaxIndexAccel, null);
                return new ShooterCollection(leftDrum, rightDrum, indexer, kMaxIndexAccel, kIndexWheelDiameterM);
                case BLANK:
                default:
                    return new ShooterCollection(
                        CollectionUtil.simulatedDriveServo(collectionLogger.child(kLeft)),
                        CollectionUtil.simulatedDriveServo(collectionLogger.child(kRight)),
                        CollectionUtil.simulatedVeclotiyMechanism(collectionLogger.child(kIndexer)),100000,2);
            }
    }

    public LinearVelocityServo[] getShooters() {
        return new LinearVelocityServo[] {
            m_leftShooter,
            m_rightShooter
        };
    }

    public LinearMechanism getIndexer() {
        return m_indexer;
    }

    public GravityServo getPivot() {
        return m_pivot;
    }

    public double getIndexerDiameterM() {
        return kIndexWheelDiameterM;
    }

    public double getIndexAccelM_S2() {
        return kMaxIndexAccelM_S2;
    }
}
