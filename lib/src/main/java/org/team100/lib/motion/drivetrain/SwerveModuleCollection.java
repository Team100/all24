package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Represents the modules in the drivetrain. */
public class SwerveModuleCollection implements SwerveModuleCollectionInterface {

    private final SwerveModule100 m_frontLeft;
    private final SwerveModule100 m_frontRight;
    private final SwerveModule100 m_rearLeft;
    private final SwerveModule100 m_rearRight;

    public SwerveModuleCollection(
            SwerveModule100 frontLeft,
            SwerveModule100 frontRight,
            SwerveModule100 rearLeft,
            SwerveModule100 rearRight) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
    }

    @Override
    public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // for testing
    @Override
    public SwerveModuleState[] getDesiredStates() {
        return new SwerveModuleState[] {
                m_frontLeft.getDesiredState(),
                m_frontRight.getDesiredState(),
                m_rearLeft.getDesiredState(),
                m_rearRight.getDesiredState()
        };
    }

    public TrapezoidProfile.State[] getSetpoint() {
        return new TrapezoidProfile.State[] {
                m_frontLeft.getSetpoint(),
                m_frontRight.getSetpoint(),
                m_rearLeft.getSetpoint(),
                m_rearRight.getSetpoint()
        };
    }

    /** For testing only */
    @Override
    public void setRawDesiredStates(SwerveModuleState[] swerveModuleStates) {
        m_frontLeft.setRawDesiredState(swerveModuleStates[0]);
        m_frontRight.setRawDesiredState(swerveModuleStates[1]);
        m_rearLeft.setRawDesiredState(swerveModuleStates[2]);
        m_rearRight.setRawDesiredState(swerveModuleStates[3]);
    }

    @Override
    public SwerveModulePosition[] positions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
    }

    @Override
    public SwerveModuleState[] states() {
        return new SwerveModuleState[] {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
        };
    }

    @Override
    public boolean[] atSetpoint() {
        return new boolean[] {
                m_frontLeft.atSetpoint(),
                m_frontRight.atSetpoint(),
                m_rearLeft.atSetpoint(),
                m_rearRight.atSetpoint()
        };
    }

    @Override
    public boolean[] atGoal() {
        return new boolean[] {
                m_frontLeft.atGoal(),
                m_frontRight.atGoal(),
                m_rearLeft.atGoal(),
                m_rearRight.atGoal()
        };
    }

    @Override
    public void periodic() {
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
    }

    @Override
    public void close() {
        m_frontLeft.close();
        m_frontRight.close();
        m_rearLeft.close();
        m_rearRight.close();
    }

    @Override
    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }
}
