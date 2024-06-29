package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Read-only view of SwerveLocal. */
public interface SwerveLocalObserver {

    SwerveModuleState[] getDesiredStates();

    /** @return current measurements */
    SwerveModuleState[] states();

    SwerveModulePosition[] positions();

    boolean[] atSetpoint();

    boolean[] atGoal();

}