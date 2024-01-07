package org.team100.lib.commands.drivetrain;

import java.util.Iterator;
import java.util.List;
import java.util.function.Function;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.FullStateDriveController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
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

/**
 * Follow a list of trajectories with the full state controller.
 * 
 * This just holds the starting rotation.  If you want a holonomic trajectory
 * follower, try the {@link DriveMotionController} classes.
 */
public class FullStateTrajectoryListCommand extends Command100 {
    private final Telemetry t = Telemetry.get();
    private final SwerveDriveSubsystem m_swerve;
    private final Timer m_timer;
    private final FullStateDriveController m_controller;
    private final Function<Pose2d, List<Trajectory>> m_trajectories;
    private Iterator<Trajectory> m_trajectoryIter;
    private Trajectory m_currentTrajectory;
    private boolean done;
    private Rotation2d m_rotation;
    private boolean m_aligned;

    public FullStateTrajectoryListCommand(
            SwerveDriveSubsystem swerve,
            Function<Pose2d, List<Trajectory>> trajectories) {
        m_swerve = swerve;
        m_controller = new FullStateDriveController();
        m_timer = new Timer();
        m_trajectories = trajectories;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        Pose2d currentPose = m_swerve.getPose();
        m_rotation = currentPose.getRotation();
        m_trajectoryIter = m_trajectories.apply(currentPose).iterator();
        m_currentTrajectory = null;
        m_timer.stop();
        m_timer.reset();
        done = false;
        m_aligned = false;
    }

    @Override
    public void execute100(double dt) {
        if (m_currentTrajectory == null || m_timer.get() > m_currentTrajectory.getTotalTimeSeconds()) {
            // get the next trajectory
            if (m_trajectoryIter.hasNext()) {
                m_currentTrajectory = m_trajectoryIter.next();
                TrajectoryVisualization.setViz(m_currentTrajectory);
                m_timer.stop();
                m_timer.reset();
                m_aligned = false;
            } else {
                done = true;
                return;
            }
        }

        // now there is a trajectory to follow
        if (m_aligned) {
            State desiredState = m_currentTrajectory.sample(m_timer.get());
            SwerveState reference = SwerveState.fromState(desiredState, m_rotation);
            SwerveState measurement = m_swerve.getState(dt);
            Twist2d fieldRelativeTarget = m_controller.calculate(measurement, reference);
            m_swerve.driveInFieldCoords(fieldRelativeTarget, dt);
            t.log(Level.DEBUG, "/full state trajectory list/reference", reference);
        } else {
            // look one loop ahead
            State desiredState = m_currentTrajectory.sample(m_timer.get() + 0.02);
            SwerveState reference = SwerveState.fromState(desiredState, m_rotation);
            SwerveState measurement = m_swerve.getState(dt);
            Twist2d fieldRelativeTarget = m_controller.calculate(measurement, reference);
            boolean aligned = m_swerve.steerAtRest(fieldRelativeTarget, dt);
            if (aligned) {
                m_aligned = true;
                m_timer.start();
            }
            t.log(Level.DEBUG, "/full state trajectory list/reference", reference);
        }

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
