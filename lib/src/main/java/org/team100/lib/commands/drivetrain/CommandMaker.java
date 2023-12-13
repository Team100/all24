package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.trajectory.StraightLineTrajectory;

import com.choreo.lib.Choreo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Utility class to produce commands.
 */
public class CommandMaker {

    /**
     * A command to follow a straight line from the current pose to the goal pose.
     */
    public static Command line(
            Experiments experiments,
            Pose2d goal,
            SwerveDriveSubsystem drivetrain,
            HolonomicDriveController3 controller,
            TrajectoryConfig config) {
        StraightLineTrajectory maker = new StraightLineTrajectory(experiments, config);
        return new DriveToWaypoint3(goal, drivetrain, maker, controller);
    }

    /**
     * Try the new Choreo library
     * 
     * see https://github.com/SleipnirGroup/Choreo/wiki/ChoreoLib-Java-Usage
     */
    public static Command choreo(SwerveDriveSubsystem drivetrain) {
        return Choreo.choreoSwerveCommand(
                Choreo.getTrajectory("test"),
                drivetrain::getPose,
                new PIDController(1, 0.0, 0.0),
                new PIDController(1, 0.0, 0.0),
                new PIDController(1, 0.0, 0.0),
                drivetrain::setChassisSpeeds,
                false,
                drivetrain);
    }

    private CommandMaker() {
        //
    }
}
