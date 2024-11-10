package org.team100.frc2024.drivetrain;

import java.util.OptionalDouble;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.util.Neo550Factory;
import org.team100.lib.util.Util;

public class TankModuleCollection {
    private static final String kSwerveModules = "Tank Modules";
    private static final String kLeft = "Left";
    private static final String kRight = "Right";
    private static final double m_5to1 = 5.2307692308;
    private static final double m_wheelDiameter = 0.098425;

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
            LoggerFactory parent,
            int currentLimit) {
        LoggerFactory collectionLogger = parent.child(kSwerveModules);
        switch (Identity.instance) {
            case DEMO_BOT:
                Util.println("************** Custom Tank Module Modules, using in built encoders? **************");
                OutboardLinearVelocityServo rightMotor = new OutboardLinearVelocityServo(collectionLogger, Neo550Factory.getNEO550LinearMechanism(
                        kRight, collectionLogger, currentLimit, 2, Math.pow(m_5to1, 2), MotorPhase.REVERSE, m_wheelDiameter));
                OutboardLinearVelocityServo leftMotor = new OutboardLinearVelocityServo(collectionLogger, Neo550Factory.getNEO550LinearMechanism(
                        kLeft, collectionLogger, currentLimit, 3, Math.pow(m_5to1, 2), MotorPhase.FORWARD, m_wheelDiameter));
                return new TankModuleCollection(leftMotor, rightMotor);
            case BLANK:
            Util.println("************** SIMULATED MODULES **************");
            default:
                return new TankModuleCollection(
                    Neo550Factory.simulatedDriveServo(collectionLogger.child(kLeft)),
                    Neo550Factory.simulatedDriveServo(collectionLogger.child(kRight)));
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
