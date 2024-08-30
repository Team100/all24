package org.team100.frc2024.drivetrain;

import java.util.OptionalDouble;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.util.CollectionUtil;
import org.team100.lib.util.Util;

public class TankModuleCollection {
    private static final String kSwerveModules = "Tank Modules";
    private static final String kLeft = "Left";
    private static final String kRight = "Right";

    private final OutboardLinearVelocityServo m_rightDrive;
    private final OutboardLinearVelocityServo m_leftDrive;

    private TankModuleCollection(OutboardLinearVelocityServo leftDrive, OutboardLinearVelocityServo rightDrive) {
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
            case FRC_100_ea4:
            case CAMERA_DOLLY:
            //TODO I am not sure if these wheel diameters are correct
                Util.println("************** Custom Tank Module Modules, using in built encoders? **************");
                OutboardLinearVelocityServo rightMotor = new OutboardLinearVelocityServo(collectionLogger, CollectionUtil.getNEO550LinearMechanism(
                        kRight, collectionLogger, currentLimit, 3, Math.pow(5.2307692308, 2), MotorPhase.REVERSE, 0.098425));
                OutboardLinearVelocityServo leftMotor = new OutboardLinearVelocityServo(collectionLogger, CollectionUtil.getNEO550LinearMechanism(
                        kLeft, collectionLogger, currentLimit, 2, Math.pow(5.2307692308, 2), MotorPhase.FORWARD, 0.098425));
                return new TankModuleCollection(leftMotor, rightMotor);
            case BLANK:
            Util.println("************** SIMULATED MODULES **************");
            default:
                return new TankModuleCollection(
                    CollectionUtil.simulatedDriveServo(collectionLogger.child(kLeft)),
                    CollectionUtil.simulatedDriveServo(collectionLogger.child(kRight)));
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
