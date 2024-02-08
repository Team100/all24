package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ChassisSpeedDriver {
    /**
     * @param input control units, [-1,1]
     * @return feasible chassis speeds in m/s and rad/s
     */
    ChassisSpeeds apply(Twist2d input);

    ChassisSpeeds apply(SwerveState state, Twist2d input);

    void reset(Pose2d p);

}
