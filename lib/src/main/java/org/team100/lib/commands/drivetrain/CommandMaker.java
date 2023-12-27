package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.TrajectoryVisualization;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/**
 * Utility class to produce commands.
 */
public class CommandMaker {

    /**
     * A command to follow a straight line from the current pose to the goal pose.
     */
    public static Command line(
            Pose2d goal,
            SwerveDriveSubsystem drivetrain,
            HolonomicDriveController3 controller,
            TrajectoryConfig config) {
        StraightLineTrajectory maker = new StraightLineTrajectory(config);
        return new DriveToWaypoint3(goal, drivetrain, maker, controller);
    }

    /**
     * Try the new Choreo library.
     * 
     * Note the fixed 0.02 sec dt here.
     * 
     * see https://github.com/SleipnirGroup/Choreo/wiki/ChoreoLib-Java-Usage
     */
    public static Command choreo(ChoreoTrajectory trajectory, SwerveDriveSubsystem drivetrain) {
        return new WrapperCommand(
                Choreo.choreoSwerveCommand(
                        trajectory,
                        drivetrain::getPose,
                        new PIDController(1, 0.0, 0.0),
                        new PIDController(1, 0.0, 0.0),
                        new PIDController(1, 0.0, 0.0),
                        x -> drivetrain.setChassisSpeeds(x, 0.02),
                        false,
                        drivetrain)) {

            @Override
            public void initialize() {
                super.initialize();
                TrajectoryVisualization.setViz(trajectory);
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                TrajectoryVisualization.clear();
            }
        };
    }

    private CommandMaker() {
        //
    }
}
