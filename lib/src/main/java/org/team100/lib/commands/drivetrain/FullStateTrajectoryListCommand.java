package org.team100.lib.commands.drivetrain;

import java.util.Iterator;
import java.util.List;
import java.util.function.Function;

import org.team100.lib.controller.FullStateDriveController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follow a list of trajectories with the full state controller.
 */
public class FullStateTrajectoryListCommand extends Command {
    private final Telemetry t = Telemetry.get();
    private final SwerveDriveSubsystemInterface m_swerve;
    private final Timer m_timer;
    private final FullStateDriveController m_controller;
    private final Function<Pose2d, List<Trajectory>> m_trajectories;
    private Iterator<Trajectory> m_trajectoryIter;
    private Trajectory m_currentTrajectory;
    private boolean done;
    // this holds the current rotation
    // TODO: allow trajectory to specify it using the new type
    private Rotation2d m_rotation;

    public FullStateTrajectoryListCommand(
            SwerveDriveSubsystemInterface swerve,
            Function<Pose2d, List<Trajectory>> trajectories) {
        m_swerve = swerve;
        m_controller = new FullStateDriveController();
        m_timer = new Timer();
        m_trajectories = trajectories;
        if (m_swerve.get() != null)
            addRequirements(m_swerve.get());
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_swerve.getPose();
        m_rotation = currentPose.getRotation();
        m_trajectoryIter = m_trajectories.apply(currentPose).iterator();
        m_currentTrajectory = null;
        m_timer.stop();
        m_timer.reset();
        done = false;
    }

    @Override
    public void execute() {
        if (m_currentTrajectory == null || m_timer.get() > m_currentTrajectory.getTotalTimeSeconds()) {
            // get the next trajectory
            if (m_trajectoryIter.hasNext()) {
                m_currentTrajectory = m_trajectoryIter.next();
                TrajectoryVisualization.setViz(m_currentTrajectory);
                m_timer.restart();
                // TODO: wheel alignment here
            } else {
                done = true;
                return;
            }
        }

        // now there is a trajectory to follow
        State desiredState = m_currentTrajectory.sample(m_timer.get());
        t.log(Level.DEBUG, "/full state trajectory list/desired state", desiredState);
        SwerveState measurement = m_swerve.getState();

        // this uses the fixed rotation.
        // TODO: rotation profile, use new trajectory type.
        SwerveState reference = SwerveState.fromState(desiredState, m_rotation);

        Twist2d fieldRelativeTarget = m_controller.calculate(measurement, reference);

        m_swerve.driveInFieldCoords(fieldRelativeTarget);

    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        TrajectoryVisualization.clear();
    }
}
