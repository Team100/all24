package org.team100.frc2024.drivetrain;

import java.util.OptionalDouble;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.util.CollectionUtil;
import org.team100.lib.util.Util;

public class TankModuleCollection {
    private static final String kSwerveModules = "Tank Modules";
    private static final String kLeft = "Left";
    private static final String kRight = "Right";

    private final LinearVelocityServo m_rightDrive;
    private final LinearVelocityServo m_leftDrive;

    private TankModuleCollection(LinearVelocityServo leftDrive, LinearVelocityServo rightDrive) {
        m_leftDrive = leftDrive;
        m_rightDrive = rightDrive;
    }

    /**
     * Creates collections according to Identity.
     */
    public static TankModuleCollection get(
            SupplierLogger parent,
            int currentLimit) {
        SupplierLogger collectionLogger = parent.child(kSwerveModules);
        switch (Identity.instance) {
            case CAMERA_DOLLY:
                Util.println("************** Custom Tank Module Modules, using in built encoders? **************");
                LinearVelocityServo leftMotor = new OutboardLinearVelocityServo(collectionLogger, CollectionUtil.getNEO550LinearMechanism(
                        kLeft, collectionLogger, currentLimit, 0, Math.pow(5.2307692308, 2), MotorPhase.FORWARD, 0.098425));
                LinearVelocityServo rightMotor = new OutboardLinearVelocityServo(collectionLogger, CollectionUtil.getNEO550LinearMechanism(
                        kRight, collectionLogger, currentLimit, 0, Math.pow(5.2307692308, 2), MotorPhase.FORWARD, 0.098425));
                return new TankModuleCollection(leftMotor, rightMotor);
            case BLANK:
            default:
                Util.println("************** SIMULATED MODULES **************");
                return new TankModuleCollection(
                    CollectionUtil.simulatedDriveServo(parent.child(kLeft)),
                    CollectionUtil.simulatedDriveServo(parent.child(kRight)));
        }
    }

    public void setDrive(double[] value) {
        m_leftDrive.setVelocityM_S(value[0]);
        m_rightDrive.setVelocityM_S(value[1]);
    }

    public OptionalDouble[] getSpeeds() {
        return new OptionalDouble[] {
            m_leftDrive.getVelocity(),
            m_rightDrive.getVelocity()
        };
    }

    public void stop() {
        m_leftDrive.stop();
        m_rightDrive.stop();
    }
}
