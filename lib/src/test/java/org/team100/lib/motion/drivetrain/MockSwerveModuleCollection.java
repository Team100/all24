package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class MockSwerveModuleCollection implements SwerveModuleCollectionInterface {
    SwerveModuleState[] m_targetModuleStates;
    boolean stopped = false;

    @Override
    public void setDesiredStates(SwerveModuleState[] targetModuleStates) {
        m_targetModuleStates = targetModuleStates;
    }

    @Override
    public SwerveModuleState[] getDesiredStates() {
        return new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
    }

    @Override
    public TrapezoidProfile.State[] getSetpoint() {
        return new TrapezoidProfile.State[] {
                new TrapezoidProfile.State(),
                new TrapezoidProfile.State(),
                new TrapezoidProfile.State(),
                new TrapezoidProfile.State()
        };
    }

    @Override
    public SwerveModulePosition[] positions() {
        return new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
    }

    @Override
    public SwerveModuleState[] states() {
        return m_targetModuleStates;
    }

    @Override
    public boolean[] atSetpoint() {
        return new boolean[] { true, true, true, true };
    }

    @Override
    public boolean[] atGoal() {
        return new boolean[] { true, true, true, true };
    }

    @Override
    public void periodic() {
        //
    }

    @Override
    public void stop() {
        stopped = true;
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void setRawDesiredStates(SwerveModuleState[] targetModuleStates) {
        m_targetModuleStates = targetModuleStates;
    }
}
