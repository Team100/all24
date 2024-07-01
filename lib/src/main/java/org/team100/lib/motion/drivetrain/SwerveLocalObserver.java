package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Read-only view of SwerveLocal. */
public interface SwerveLocalObserver {

    SwerveModuleState[] getDesiredStates();

    /** @return current measurements */
    SwerveModuleState[] states();

    SwerveModulePosition[] positions();

    boolean[] atSetpoint();

    boolean[] atGoal();

}