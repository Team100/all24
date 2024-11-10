package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;


/** Read-only view of SwerveLocal. */
public interface SwerveLocalObserver {

    SwerveModuleStates getDesiredStates();

    /** @return current measurements */
    SwerveModuleStates states();

    SwerveModulePositions positions();

    boolean[] atSetpoint();

    boolean[] atGoal();

}