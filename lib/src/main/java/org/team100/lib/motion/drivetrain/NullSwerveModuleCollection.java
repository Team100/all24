package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * An implementation that does nothing, for cases where there is no physical
 * drivetrain.
 */
public class NullSwerveModuleCollection implements SwerveModuleCollectionInterface {
    @Override
    public void setDesiredStates(SwerveModuleState[] targetModuleStates) {
        //
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
    public void close() {
        //
    }

    @Override
    public SwerveModuleState[] states() {
        return new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
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
        //
    }

    @Override
    public void setRawDesiredStates(SwerveModuleState[] targetModuleStates) {
        //
    }

}
