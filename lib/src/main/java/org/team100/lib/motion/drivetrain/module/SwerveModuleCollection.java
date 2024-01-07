package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.State100;
import org.team100.lib.encoder.turning.AnalogTurningEncoder.Drive;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Util;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Represents the modules in the drivetrain. */
public class SwerveModuleCollection {
    // these names should not have slashes, it messes up the visualization
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
            double currentLimit,
            SwerveKinodynamics kinodynamics) {
        switch (Identity.instance) {
            case COMP_BOT:
                Util.println("************** WCP MODULES **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.get(
                                kFrontLeft, currentLimit, 11, 30, 0, 0.708328, kinodynamics),
                        WCPSwerveModule100.get(
                                kFrontRight, currentLimit, 12, 32, 1, 0.659267, kinodynamics),
                        WCPSwerveModule100.get(
                                kRearLeft, currentLimit, 21, 31, 2, 0.396148, kinodynamics),
                        WCPSwerveModule100.get(
                                kRearRight, currentLimit, 22, 33, 3, 0.109823, kinodynamics));
            case SWERVE_TWO:
                Util.println("************** AM CAN MODULES **************");
                return new SwerveModuleCollection(
                        AMCANSwerveModule100.get(
                                kFrontLeft, currentLimit, 3, 36, 2, 0.354994, Drive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(
                                kFrontRight, currentLimit, 12, 13, 3, 0.880423, Drive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(
                                kRearLeft, currentLimit, 22, 1, 1, 0.916801, Drive.INVERSE, kinodynamics),
                        AMCANSwerveModule100.get(
                                kRearRight, currentLimit, 21, 0, 0, 0.806963, Drive.INVERSE, kinodynamics));
            case SWERVE_ONE:
                Util.println("************** AM CAN MODULES **************");
                return new SwerveModuleCollection(
                        AMCANSwerveModule100.get(
                                kFrontLeft, currentLimit, 11, 5, 2, 0.694815, Drive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(
                                kFrontRight, currentLimit, 12, 2, 0, 0.718789, Drive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(
                                kRearLeft, currentLimit, 21, 3, 3, 0.365612, Drive.DIRECT, kinodynamics),
                        AMCANSwerveModule100.get(
                                kRearRight, currentLimit, 22, 1, 1, 0.942851, Drive.DIRECT, kinodynamics));
            case BLANK:
                Util.println("************** SIMULATED MODULES **************");
                return new SwerveModuleCollection(
                        SimulatedSwerveModule100.get(kFrontLeft, kinodynamics),
                        SimulatedSwerveModule100.get(kFrontRight, kinodynamics),
                        SimulatedSwerveModule100.get(kRearLeft, kinodynamics),
                        SimulatedSwerveModule100.get(kRearRight, kinodynamics));
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

    public void periodic() {
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
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
