package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.async.Async;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.State100;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.encoder.turning.EncoderDrive;
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
            SwerveKinodynamics kinodynamics,
            Async async) {
        switch (Identity.instance) {
            case COMP_BOT:
                Util.println("************** WCP MODULES w/Duty-Cycle Encoders **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.get(kFrontLeft, currentLimit, statorLimit,
                                4,
                                DriveRatio.FAST, DutyCycleTurningEncoder.class,
                                54,
                                9,
                                0.058735,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.get(kFrontRight, currentLimit, statorLimit,
                                22,
                                DriveRatio.FAST, DutyCycleTurningEncoder.class,
                                52,
                                8,
                                0.773486,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.get(kRearLeft, currentLimit, statorLimit,
                                56,
                                DriveRatio.FAST, DutyCycleTurningEncoder.class,
                                51,
                                5,
                                0.334580,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.get(kRearRight, currentLimit, statorLimit,
                                11,
                                DriveRatio.FAST, DutyCycleTurningEncoder.class,
                                21,
                                7,
                                0.714328,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE));
            case BETA_BOT:
                Util.println("************** WCP MODULES w/Analog Encoders **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.get(kFrontLeft, currentLimit, statorLimit,
                                30,
                                DriveRatio.FAST, AnalogTurningEncoder.class,
                                11,
                                0,
                                0.620381,
                                kinodynamics,
                                EncoderDrive.DIRECT, MotorPhase.REVERSE),
                        WCPSwerveModule100.get(kFrontRight, currentLimit, statorLimit,
                                12,
                                DriveRatio.FAST, AnalogTurningEncoder.class,
                                32,
                                1,
                                0.162019,
                                kinodynamics,
                                EncoderDrive.DIRECT, MotorPhase.REVERSE),
                        WCPSwerveModule100.get(kRearLeft, currentLimit, statorLimit,
                                31,
                                DriveRatio.FAST, AnalogTurningEncoder.class,
                                21,
                                2,
                                0.875648,
                                kinodynamics,
                                EncoderDrive.DIRECT, MotorPhase.REVERSE),
                        WCPSwerveModule100.get(kRearRight, currentLimit, statorLimit,
                                22,
                                DriveRatio.FAST, AnalogTurningEncoder.class,
                                33,
                                3,
                                0.323889,
                                kinodynamics,
                                EncoderDrive.DIRECT, MotorPhase.REVERSE));
            case SWERVE_ONE:
                Util.println("************** AM CAN MODULES **************");
                return new SwerveModuleCollection(
                        AMCANSwerveModule100.get(kFrontLeft, currentLimit, statorLimit,
                                11,
                                5,
                                2,
                                0.694815,
                                EncoderDrive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(kFrontRight, currentLimit, statorLimit,
                                12,
                                2,
                                0,
                                0.718789,
                                EncoderDrive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(kRearLeft, currentLimit, statorLimit,
                                21,
                                3,
                                3,
                                0.365612,
                                EncoderDrive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(kRearRight, currentLimit, statorLimit,
                                22,
                                1,
                                1,
                                0.942851,
                                EncoderDrive.DIRECT, kinodynamics));
            case SWERVE_TWO:
                Util.println("************** AM CAN MODULES **************");
                return new SwerveModuleCollection(
                        AMCANSwerveModule100.get(kFrontLeft, currentLimit, statorLimit,
                                3,
                                36,
                                2,
                                0.354994,
                                EncoderDrive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(kFrontRight, currentLimit, statorLimit,
                                12,
                                13,
                                3,
                                0.880423,
                                EncoderDrive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(kRearLeft, currentLimit, statorLimit,
                                22,
                                1,
                                1,
                                0.916801,
                                EncoderDrive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(kRearRight, currentLimit, statorLimit,
                                21,
                                0,
                                0,
                                0.806963,
                                EncoderDrive.INVERSE, kinodynamics));
            case BLANK:
            default:
                Util.println("************** SIMULATED MODULES **************");
                return new SwerveModuleCollection(
                        SimulatedSwerveModule100.get(kFrontLeft, kinodynamics),
                        SimulatedSwerveModule100.get(kFrontRight, kinodynamics),
                        SimulatedSwerveModule100.get(kRearLeft, kinodynamics),
                        SimulatedSwerveModule100.get(kRearRight, kinodynamics));
        }
    }

    //////////////////////////////////////////////////
    //
    // Actuators
    //

    public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void setRawDesiredStates(SwerveModuleState[] swerveModuleStates) {
        m_frontLeft.setRawDesiredState(swerveModuleStates[0]);
        m_frontRight.setRawDesiredState(swerveModuleStates[1]);
        m_rearLeft.setRawDesiredState(swerveModuleStates[2]);
        m_rearRight.setRawDesiredState(swerveModuleStates[3]);
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

    //////////////////////////////////////////////////////
    //
    // Observers
    //

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

    public SwerveModulePosition[] positions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
    }

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

    ////////////////////////////////////////////

    public void close() {
        m_frontLeft.close();
        m_frontRight.close();
        m_rearLeft.close();
        m_rearRight.close();
    }

    public SwerveModule100[] modules() {
        return new SwerveModule100[] {
                m_frontLeft,
                m_frontRight,
                m_rearLeft,
                m_rearRight };
    }
}
