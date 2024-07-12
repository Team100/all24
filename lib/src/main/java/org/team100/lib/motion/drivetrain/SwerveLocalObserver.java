package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;


/** Read-only view of SwerveLocal. */
public interface SwerveLocalObserver {

    SwerveModuleState100[] getDesiredStates();

    /** @return current measurements */
    SwerveModuleState100[] states();

    SwerveModulePosition100[] positions();

    boolean[] atSetpoint();

    boolean[] atGoal();

}