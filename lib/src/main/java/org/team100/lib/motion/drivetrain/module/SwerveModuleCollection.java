package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.AnalogTurningEncoder;
import org.team100.lib.encoder.DutyCycleRotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.motion.drivetrain.module.WCPSwerveModule100.DriveRatio;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.state.State100;
import org.team100.lib.util.Util;

/** Represents the modules in the drivetrain. */
public class SwerveModuleCollection {
    private static final String kSwerveModules = "Swerve Modules";
    private static final String kFrontLeft = "Front Left";
    private static final String kFrontRight = "Front Right";
    private static final String kRearLeft = "Rear Left";
    private static final String kRearRight = "Rear Right";

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
            LoggerFactory parent,
            double currentLimit,
            double statorLimit,
            SwerveKinodynamics kinodynamics) {
        LoggerFactory collectionLogger = parent.child(kSwerveModules);
        LoggerFactory frontLeftLogger = collectionLogger.child(kFrontLeft);
        LoggerFactory frontRightLogger = collectionLogger.child(kFrontRight);
        LoggerFactory rearLeftLogger = collectionLogger.child(kRearLeft);
        LoggerFactory rearRightLogger = collectionLogger.child(kRearRight);

        switch (Identity.instance) {
            case COMP_BOT:
                Util.println("************** WCP MODULES w/Duty-Cycle Encoders **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.getKrakenDrive(frontLeftLogger,
                                currentLimit,
                                statorLimit,
                                4,
                                DriveRatio.FAST, DutyCycleRotaryPositionSensor.class,
                                54,
                                9,
                                0.058735,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(frontRightLogger,
                                currentLimit,
                                statorLimit,
                                22,
                                DriveRatio.FAST, DutyCycleRotaryPositionSensor.class,
                                52,
                                8,
                                0.773486,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(rearLeftLogger,
                                currentLimit,
                                statorLimit,
                                56,
                                DriveRatio.FAST, DutyCycleRotaryPositionSensor.class,
                                51,
                                5,
                                0.334580,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(rearRightLogger,
                                currentLimit,
                                statorLimit,
                                11,
                                DriveRatio.FAST, DutyCycleRotaryPositionSensor.class,
                                21,
                                7,
                                0.714328,
                                kinodynamics,
                                EncoderDrive.INVERSE, MotorPhase.REVERSE));
            case SWERVE_ONE:
                Util.println("************** WCP MODULES w/Analog Encoders **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.getFalconDrive(frontLeftLogger,
                                currentLimit,
                                statorLimit,
                                32,
                                DriveRatio.FAST, AnalogTurningEncoder.class,
                                12,
                                2,
                                0.336,
                                kinodynamics,
                                EncoderDrive.DIRECT, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(frontRightLogger,
                                currentLimit,
                                statorLimit,
                                30,
                                DriveRatio.FAST, AnalogTurningEncoder.class,
                                11,
                                1,
                                0.619,
                                kinodynamics,
                                EncoderDrive.DIRECT, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(rearLeftLogger,
                                currentLimit,
                                statorLimit,
                                31,
                                DriveRatio.FAST, AnalogTurningEncoder.class,
                                21,
                                3,
                                0.591,
                                kinodynamics,
                                EncoderDrive.DIRECT, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(rearRightLogger,
                                currentLimit,
                                statorLimit,
                                22,
                                DriveRatio.FAST, AnalogTurningEncoder.class,
                                33,
                                0,
                                0.512,
                                kinodynamics,
                                EncoderDrive.DIRECT, MotorPhase.REVERSE));
            case BETA_BOT:
                Util.println("************** AM CAN MODULES **************");
                return new SwerveModuleCollection(
                        AMCANSwerveModule100.get(frontLeftLogger,
                                currentLimit,
                                statorLimit,
                                11,
                                5,
                                2,
                                0.694815,
                                EncoderDrive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(frontRightLogger,
                                currentLimit,
                                statorLimit,
                                12,
                                2,
                                0,
                                0.718789,
                                EncoderDrive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(rearLeftLogger,
                                currentLimit,
                                statorLimit,
                                21,
                                3,
                                3,
                                0.365612,
                                EncoderDrive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(rearRightLogger,
                                currentLimit,
                                statorLimit,
                                22,
                                1,
                                1,
                                0.942851,
                                EncoderDrive.DIRECT, kinodynamics));
            case SWERVE_TWO:
                Util.println("************** AM CAN MODULES **************");
                return new SwerveModuleCollection(
                        AMCANSwerveModule100.get(frontLeftLogger,
                                currentLimit,
                                statorLimit,
                                3,
                                36,
                                2,
                                0.354994,
                                EncoderDrive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(frontRightLogger,
                                currentLimit,
                                statorLimit,
                                12,
                                13,
                                3,
                                0.880423,
                                EncoderDrive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(rearLeftLogger,
                                currentLimit,
                                statorLimit,
                                22,
                                1,
                                1,
                                0.916801,
                                EncoderDrive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(rearRightLogger,
                                currentLimit,
                                statorLimit,
                                21,
                                0,
                                0,
                                0.806963,
                                EncoderDrive.INVERSE, kinodynamics));
            case BLANK:
            default:
                Util.println("************** SIMULATED MODULES **************");
                return new SwerveModuleCollection(
                        SimulatedSwerveModule100.get(frontLeftLogger, kinodynamics),
                        SimulatedSwerveModule100.get(frontRightLogger, kinodynamics),
                        SimulatedSwerveModule100.get(rearLeftLogger, kinodynamics),
                        SimulatedSwerveModule100.get(rearRightLogger, kinodynamics));
        }
    }

    //////////////////////////////////////////////////
    //
    // Actuators
    //


    public void setDesiredStates(SwerveModuleStates swerveModuleStates) {
        m_frontLeft.setDesiredState(swerveModuleStates.frontLeft());
        m_frontRight.setDesiredState(swerveModuleStates.frontRight());
        m_rearLeft.setDesiredState(swerveModuleStates.rearLeft());
        m_rearRight.setDesiredState(swerveModuleStates.rearRight());
    }

    public void setRawDesiredStates(SwerveModuleStates swerveModuleStates) {
        m_frontLeft.setRawDesiredState(swerveModuleStates.frontLeft());
        m_frontRight.setRawDesiredState(swerveModuleStates.frontRight());
        m_rearLeft.setRawDesiredState(swerveModuleStates.rearLeft());
        m_rearRight.setRawDesiredState(swerveModuleStates.rearRight());
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

    public SwerveModuleStates getDesiredStates() {
        return new SwerveModuleStates(
                m_frontLeft.getDesiredState(),
                m_frontRight.getDesiredState(),
                m_rearLeft.getDesiredState(),
                m_rearRight.getDesiredState());
    }

    public State100[] getSetpoint() {
        return new State100[] {
                m_frontLeft.getSetpoint(),
                m_frontRight.getSetpoint(),
                m_rearLeft.getSetpoint(),
                m_rearRight.getSetpoint()
        };
    }

    public SwerveModulePositions positions() {
        return new SwerveModulePositions(
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition());
    }

    public SwerveModuleStates states() {
        return new SwerveModuleStates(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
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

    /** Updates visualization. */
    public void periodic() {
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
    }
}
