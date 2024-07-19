package org.team100.frc2024.drivetrain;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.SimpleLinearMechanism;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.telemetry.SupplierLogger;
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
            LinearVelocityServo leftMotor = TankModule100.get(
                kLeft, collectionLogger, currentLimit, 0, 5, MotorPhase.FORWARD);
                LinearVelocityServo rightMotor = TankModule100.get(
                    kRight, collectionLogger, currentLimit, 0, 5, MotorPhase.FORWARD);
                return new TankModuleCollection(leftMotor, rightMotor);
            case BLANK:
            default:
            Util.println("************** SIMULATED MODULES **************");
            return new TankModuleCollection(
                simulatedDriveServo(parent.child(kLeft)), 
                simulatedDriveServo(parent.child(kRight)));
        }
    }

    private static LinearVelocityServo simulatedDriveServo(SupplierLogger parent) {
        // simulated drive motor free speed is 5 m/s
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        // simulated gearing is 2 meter wheel, 1:1, so rad/s and m/s are the same.
        LinearMechanism mech = new SimpleLinearMechanism(
                driveMotor,
                new SimulatedBareEncoder(parent, driveMotor),
                1,
                2);
        return new OutboardLinearVelocityServo(
                parent,
                mech);
    }

    public void setDrive(double[] value){
        m_rightDrive.setVelocityM_S(value[0]);
        m_leftDrive.setVelocityM_S(value[1]);
    }

    public void stop() {
        m_leftDrive.stop();
        m_rightDrive.stop();
    }
}
