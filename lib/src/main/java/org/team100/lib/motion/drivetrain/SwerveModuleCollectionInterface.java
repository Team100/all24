package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface SwerveModuleCollectionInterface {

    void setDesiredStates(SwerveModuleState[] targetModuleStates);

    /** For testing only */
    SwerveModuleState[] getDesiredStates();
    
    /** For testing only */
    TrapezoidProfile.State[] getSetpoint();
    
    /** For testing only */
    void setRawDesiredStates(SwerveModuleState[] targetModuleStates);

    SwerveModulePosition[] positions();

    SwerveModuleState[] states();

    boolean[] atSetpoint();

    boolean[] atGoal();

    /** Update the visualization. */
    void periodic();

    void stop();

    void close();
}
