package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.State100;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.encoder.turning.Drive;
import org.team100.lib.encoder.turning.DutyCycleTurningEncoder;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.module.WCPSwerveModule100.DriveRatio;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.util.Util;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Represents the modules in the drivetrain. */
public class SwerveModuleCollection {
    private static final String kSwerveModules = "Swerve Modules";
    private static final String kFrontLeft = kSwerveModules + "/Front Left";
    private static final String kFrontRight = kSwerveModules + "/Front Right";
    private static final String kRearLeft = kSwerveModules + "/Rear Left";
    private static final String kRearRight = kSwerveModules + "/Rear Right";
    // private static final String kFrontLeft = "Front Left";
    // private static final String kFrontRight = "Front Right";
    // private static final String kRearLeft = "Rear Left";
    // private static final String kRearRight = "Rear Right";

    private final SwerveModule100 m_frontLeft;
    private final SwerveModule100 m_frontRight;
    private final SwerveModule100 m_rearLeft;
    private final SwerveModule100 m_rearRight;

    private SwerveModuleCollection(
            SwerveModule100 frontLeft,
            SwerveModule100 frontRight,
            SwerveModule100 rearLeft,
            SwerveModule100 rearRight) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
    }

    /**
     * Creates collections according to Identity.
     */
    public static SwerveModuleCollection get(
            double currentLimit,
            double statorLimit,
            SwerveKinodynamics kinodynamics) {
        switch (Identity.instance) {
            case BETA_BOT:
                Util.println("************** WCP MODULES using AI 0,1,2,3 **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.get(
                                kFrontLeft, currentLimit, statorLimit, 30, DriveRatio.FAST,
                                AnalogTurningEncoder.class, 11, 0, 0.620381, kinodynamics, Drive.DIRECT,
                                MotorPhase.REVERSE),
                        WCPSwerveModule100.get(
                                kFrontRight, currentLimit, statorLimit, 12, DriveRatio.FAST,
                                AnalogTurningEncoder.class, 32, 1, 0.162019, kinodynamics, Drive.DIRECT,
                                MotorPhase.REVERSE),
                        WCPSwerveModule100.get(
                                kRearLeft, currentLimit, statorLimit, 31, DriveRatio.FAST,
                                AnalogTurningEncoder.class, 21, 2, 0.875648, kinodynamics, Drive.DIRECT,
                                MotorPhase.REVERSE),
                        WCPSwerveModule100.get(
                                kRearRight, currentLimit, statorLimit, 22, DriveRatio.FAST,
                                AnalogTurningEncoder.class, 33, 3, 0.323889, kinodynamics, Drive.DIRECT,
                                MotorPhase.REVERSE));
            case SWERVE_TWO:
                Util.println("************** AM CAN MODULES **************");
                return new SwerveModuleCollection(
                        AMCANSwerveModule100.get(
                                kFrontLeft, currentLimit, statorLimit, 3,
                                36, 2, 0.354994, Drive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(
                                kFrontRight, currentLimit, statorLimit, 12,
                                13, 3, 0.880423, Drive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(
                                kRearLeft, currentLimit, statorLimit, 22,
                                1, 1, 0.916801, Drive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(
                                kRearRight, currentLimit, statorLimit, 21,
                                0, 0, 0.806963, Drive.INVERSE, kinodynamics));
            case SWERVE_ONE:
                Util.println("************** AM CAN MODULES **************");
                return new SwerveModuleCollection(
                        AMCANSwerveModule100.get(
                                kFrontLeft, currentLimit, statorLimit, 11,
                                5, 2, 0.694815, Drive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(
                                kFrontRight, currentLimit, statorLimit, 12,
                                2, 0, 0.718789, Drive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(
                                kRearLeft, currentLimit, statorLimit, 21,
                                3, 3, 0.365612, Drive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(
                                kRearRight, currentLimit, statorLimit, 22,
                                1, 1, 0.942851, Drive.DIRECT, kinodynamics));
            case BLANK:
                Util.println("************** SIMULATED MODULES **************");
                return new SwerveModuleCollection(
                        SimulatedSwerveModule100.get(kFrontLeft, kinodynamics),
                        SimulatedSwerveModule100.get(kFrontRight, kinodynamics),
                        SimulatedSwerveModule100.get(kRearLeft, kinodynamics),
                        SimulatedSwerveModule100.get(kRearRight, kinodynamics));
            case COMP_BOT:
                Util.println("************** WCP MODULES using DIO 0,1,2,3 **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.get(
                                kFrontLeft, currentLimit, statorLimit, 4, WCPSwerveModule100.DriveRatio.FAST,
                                DutyCycleTurningEncoder.class,
                                54,
                                9,
                                0.058735,
                                kinodynamics, Drive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.get(
                                kFrontRight, currentLimit, statorLimit, 22, WCPSwerveModule100.DriveRatio.FAST,
                                DutyCycleTurningEncoder.class,
                                52,
                                8,
                                0.773486,
                                kinodynamics, Drive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.get(
                                kRearLeft, currentLimit, statorLimit, 56, WCPSwerveModule100.DriveRatio.FAST,
                                DutyCycleTurningEncoder.class,
                                51,
                                6,
                                0.334580,
                                kinodynamics, Drive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.get(
                                kRearRight, currentLimit, statorLimit, 11, WCPSwerveModule100.DriveRatio.FAST,
                                DutyCycleTurningEncoder.class,
                                21,
                                7,
                                0.714328,
                                kinodynamics, Drive.INVERSE, MotorPhase.REVERSE));
            default:
                Util.println("WARNING: using default module collection");
                return new SwerveModuleCollection(
                        SimulatedSwerveModule100.get(kFrontLeft, kinodynamics),
                        SimulatedSwerveModule100.get(kFrontRight, kinodynamics),
                        SimulatedSwerveModule100.get(kRearLeft, kinodynamics),
                        SimulatedSwerveModule100.get(kRearRight, kinodynamics));
        }
    }

    public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // for testing
    public SwerveModuleState[] getDesiredStates() {
        return new SwerveModuleState[] {
                m_frontLeft.getDesiredState(),
                m_frontRight.getDesiredState(),
                m_rearLeft.getDesiredState(),
                m_rearRight.getDesiredState()
        };
    }

    public State100[] getSetpoint() {
        return new State100[] {
                m_frontLeft.getSetpoint(),
                m_frontRight.getSetpoint(),
                m_rearLeft.getSetpoint(),
                m_rearRight.getSetpoint()
        };
    }

    /** For testing only */
    public void setRawDesiredStates(SwerveModuleState[] swerveModuleStates) {
        m_frontLeft.setRawDesiredState(swerveModuleStates[0]);
        m_frontRight.setRawDesiredState(swerveModuleStates[1]);
        m_rearLeft.setRawDesiredState(swerveModuleStates[2]);
        m_rearRight.setRawDesiredState(swerveModuleStates[3]);
    }

    public SwerveModulePosition[] positions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
    }

    /** @return current measurements */
    public SwerveModuleState[] states() {
        return new SwerveModuleState[] {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
        };
    }

    public boolean[] atSetpoint() {
        return new boolean[] {
                m_frontLeft.atSetpoint(),
                m_frontRight.atSetpoint(),
                m_rearLeft.atSetpoint(),
                m_rearRight.atSetpoint()
        };
    }

    public boolean[] atGoal() {
        return new boolean[] {
                m_frontLeft.atGoal(),
                m_frontRight.atGoal(),
                m_rearLeft.atGoal(),
                m_rearRight.atGoal()
        };
    }

    public void close() {
        m_frontLeft.close();
        m_frontRight.close();
        m_rearLeft.close();
        m_rearRight.close();
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    public void reset() {
        m_frontLeft.reset();
        m_frontRight.reset();
        m_rearLeft.reset();
        m_rearRight.reset();
    }
}
