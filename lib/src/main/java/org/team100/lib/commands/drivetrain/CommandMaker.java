package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.visualization.TrajectoryVisualization;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/**
 * Utility class to produce commands.
 */
public class CommandMaker {

    /**
     * Try the new Choreo library.
     * 
     * Note the fixed 0.02 sec dt here.
     * 
     * see https://github.com/SleipnirGroup/Choreo/wiki/ChoreoLib-Java-Usage
     */
    public static Command choreo(
        ChoreoTrajectory trajectory, 
        SwerveDriveSubsystem drivetrain,
        TrajectoryVisualization viz) {
        return new WrapperCommand(
                Choreo.choreoSwerveCommand(
                        trajectory,
                        () -> drivetrain.getState().pose(),
                        new PIDController(0.25, 0.0, 0.0),
                        new PIDController(0.25, 0.0, 0.0),
                        new PIDController(0.25, 0.0, 0.0),
                        x -> drivetrain.setChassisSpeeds(x, 0.02),
                        () -> false,
                        drivetrain)) {

            @Override
            public void initialize() {
                super.initialize();
                viz.setViz(trajectory);
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                viz.clear();
            }
        };
    }

    private CommandMaker() {
        //
    }
}
