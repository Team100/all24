package org.team100.frc2024.shooter.pivot;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.util.Neo550Factory;

public class PivotCollection {
    private static final String kPivot = "Pivot";

    private final OutboardGravityServo m_pivot;

    private PivotCollection(OutboardGravityServo pivot) {
        m_pivot = pivot;
    }

    public static PivotCollection get(
            LoggerFactory parent,
            int currentLimit
            ) {
        LoggerFactory collectionLogger = parent.child(kPivot);
        switch (Identity.instance) {
            case DEMO_BOT:
            //TODO get canID, gearRatio, p values, and gravityNm
            OutboardGravityServo pivot = Neo550Factory.getNEO550GravityServo(kPivot, collectionLogger, currentLimit, 1, 14, MotorPhase.FORWARD,1,0.1,0,-Math.PI/2, 0);
            return new PivotCollection(pivot);
            case BLANK:
            default:
            return new PivotCollection(Neo550Factory.simulatedGravityServo(collectionLogger));
        }
    }

    public OutboardGravityServo getPivot() {
        return m_pivot;
    }
}
