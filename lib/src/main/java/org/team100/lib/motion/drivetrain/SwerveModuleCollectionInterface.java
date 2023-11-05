package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleCollectionInterface {

    SwerveModulePosition[] positions();

    void close();

    SwerveModuleState[] states();

    void stop();

    void test(double[][] desiredOutputs);

    void setDesiredStates(SwerveModuleState[] targetModuleStates);
    
}
