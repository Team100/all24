package org.team100.frc2024.shooter;

import java.util.Optional;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.components.GravityServo;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OutboardAngularPositionServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.util.CollectionUtil;
import edu.wpi.first.wpilibj.Servo;

public class ShooterCollection {
    private static final String kShooter = "Shooter Wheels";
    private static final String kLeft = "Left";
    private static final String kRight = "Right";
    private static final String kPivot = "Pivot";

    private final LinearVelocityServo m_leftShooter;
    private final LinearVelocityServo m_rightShooter;
    private final Servo m_indexer;
    private final GravityServo m_pivot;

    private ShooterCollection(LinearVelocityServo leftShooter, LinearVelocityServo rightShooter, Servo indexer, GravityServo pivot) {
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
                case CAMERA_DOLLY:
                //TODO get the real diameter, gearRatios, and canIDs, and Indexer accel
                LinearVelocityServo leftDrum = CollectionUtil.getNEO550VelocityServo(kLeft, collectionLogger,currentLimit,4,5.2307692308,MotorPhase.FORWARD, shooterWheelDiameterM);
                LinearVelocityServo rightDrum = CollectionUtil.getNEO550VelocityServo(kRight, collectionLogger,currentLimit,5,5.2307692308,MotorPhase.REVERSE, shooterWheelDiameterM);
                Servo indexer = new Servo(1);
                GravityServo pivot = new GravityServo(new OutboardAngularPositionServo(collectionLogger,CollectionUtil.getNEO550RotaryMechanism(kPivot, collectionLogger, currentLimit, 1, 5.2307692308,MotorPhase.FORWARD), Optional.empty()),1,0);
                return new ShooterCollection(leftDrum, rightDrum, indexer,pivot);
                case BLANK:
                default:
                    return new ShooterCollection(
                        CollectionUtil.simulatedDriveServo(collectionLogger.child(kLeft)),
                        CollectionUtil.simulatedDriveServo(collectionLogger.child(kRight)),
                        new Servo(1),
                        new GravityServo(new OutboardAngularPositionServo(collectionLogger,CollectionUtil.simulatedRotaryMechanism(collectionLogger),Optional.empty()),1,0));
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

    public GravityServo getPivot() {
        return m_pivot;
    }
}
