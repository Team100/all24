package org.team100.lib.commands.drivetrain;

import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.HolonomicFieldRelativeController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.trajectory.TrajectoryVisualization;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Similar to TrajectoryListCommand, but each trajectory starts wherever the
 * robot ends up, instead of at the end of the previous trajectory. This is
 * essentially like ignoring cross-track error.
 */
public class PermissiveTrajectoryListCommand extends Command100 {
    private final Telemetry t = Telemetry.get();
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicFieldRelativeController m_controller;
    private final List<Function<Pose2d, Trajectory100>> m_trajectories;
    private Iterator<Function<Pose2d, Trajectory100>> m_trajectoryIter;
    private Trajectory100 m_currentTrajectory;
    private TrajectoryTimeIterator m_iter;
    private boolean done;
    private boolean m_aligned;

    public PermissiveTrajectoryListCommand(
            SwerveDriveSubsystem swerve,
            HolonomicFieldRelativeController controller,
            List<Function<Pose2d, Trajectory100>> trajectories) {
        m_swerve = swerve;
        m_controller = controller;
        m_trajectories = trajectories;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        m_controller.reset();
        m_trajectoryIter = m_trajectories.iterator();
        m_currentTrajectory = null;
        done = false;
        m_aligned = false;
    }

    @Override
    public void execute100(double dt) {
        if (m_currentTrajectory == null || m_iter.isDone()) {
            // get the next trajectory
            if (m_trajectoryIter.hasNext()) {
                m_currentTrajectory = m_trajectoryIter.next().apply(m_swerve.getPose());
                m_iter = new TrajectoryTimeIterator(
                        new TrajectoryTimeSampler(m_currentTrajectory));
                TrajectoryVisualization.setViz(m_currentTrajectory);
                m_aligned = false;
            } else {
                done = true;
                return;
            }
        }

        // now there is a trajectory to follow

        if (m_aligned) {
            Optional<TrajectorySamplePoint> optSamplePoint = m_iter.advance(dt);
            if (optSamplePoint.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TrajectorySamplePoint samplePoint = optSamplePoint.get();
            TimedPose desiredState = samplePoint.state();

            Pose2d currentPose = m_swerve.getPose();
            SwerveState reference = SwerveState.fromTimedPose(desiredState);
            t.log(Level.TRACE, m_name, "reference", reference);
            Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);
            m_swerve.driveInFieldCoords(fieldRelativeTarget, dt);
        } else {
            // look just one loop ahead by *previewing* the next point
            Optional<TrajectorySamplePoint> optSamplePoint = m_iter.preview(dt);
            if (optSamplePoint.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TrajectorySamplePoint samplePoint = optSamplePoint.get();
            TimedPose desiredState = samplePoint.state();

            Pose2d currentPose = m_swerve.getPose();
            SwerveState reference = SwerveState.fromTimedPose(desiredState);
            t.log(Level.TRACE, m_name, "reference", reference);
            Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);
            m_aligned = m_swerve.steerAtRest(fieldRelativeTarget, dt);
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
