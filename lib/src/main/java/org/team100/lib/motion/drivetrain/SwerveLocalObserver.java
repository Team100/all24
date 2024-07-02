package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Read-only view of SwerveLocal. */
public interface SwerveLocalObserver {

    SwerveModuleState100[] getDesiredStates();

    /** @return current measurements */
    SwerveModuleState100[] states();

    SwerveModulePosition[] positions();

    boolean[] atSetpoint();

    boolean[] atGoal();

}