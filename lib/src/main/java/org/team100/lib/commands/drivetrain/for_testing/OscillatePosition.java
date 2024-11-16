package org.team100.lib.commands.drivetrain.for_testing;

import java.util.Optional;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Choose a position 1m away, go there, then go back, endlessly.
 * It's like DriveToWaypoint3, but with a robot-relative goal set in
 * initialize().
 * 
 * this is for testing odometry.
 */
public class OscillatePosition extends Command implements Glassy {

    private final SwerveDriveSubsystem m_swerve;
    private final StraightLineTrajectory m_trajectories;
    private final HolonomicFieldRelativeController m_controller;
    private final double m_offsetM;
    private final DoubleLogger m_log_trajecX;
    private final DoubleLogger m_log_trajecY;

    private Trajectory100 m_trajectory;
    private TrajectoryTimeIterator m_iter;
    private final TrajectoryVisualization m_viz;

    private boolean m_steeringAligned;

    public OscillatePosition(
            LoggerFactory parent,
            SwerveDriveSubsystem drivetrain,
            StraightLineTrajectory trajectories,
            HolonomicFieldRelativeController controller,
            double offsetM,
            TrajectoryVisualization viz) {
        LoggerFactory child = parent.child(this);
        m_swerve = drivetrain;
        m_trajectories = trajectories;
        m_controller = controller;
        m_viz = viz;
        m_offsetM = offsetM;
        m_log_trajecX = child.doubleLogger(Level.TRACE, "Trajec X");
        m_log_trajecY = child.doubleLogger(Level.TRACE, "Trajec Y");
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_controller.reset();
        // choose a goal 1m away
        SwerveModel start = m_swerve.getState();
        Pose2d startPose = start.pose();
        Pose2d endPose = startPose.plus(new Transform2d(m_offsetM, 0, new Rotation2d()));
        m_trajectory = m_trajectories.apply(start, endPose);
        m_iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(m_trajectory));
        m_steeringAligned = false;
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
        if (m_trajectory == null)
            return;
        SwerveModel measurement = m_swerve.getState();

        if (m_steeringAligned) {
            Optional<TrajectorySamplePoint> optSamplePoint = m_iter.advance(TimedRobot100.LOOP_PERIOD_S);
            if (optSamplePoint.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TrajectorySamplePoint samplePoint = optSamplePoint.get();

            TimedPose desiredState = samplePoint.state();
            SwerveModel reference = SwerveModel.fromTimedPose(desiredState);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(measurement, reference);
            m_log_trajecX.log(() -> desiredState.state().getPose().getX());
            m_log_trajecY.log(() -> desiredState.state().getPose().getY());

            // follow normally
            m_swerve.driveInFieldCoords(fieldRelativeTarget);
        } else {
            // not aligned yet, try aligning by *previewing* next point
            Optional<TrajectorySamplePoint> optSamplePoint = m_iter.preview(TimedRobot100.LOOP_PERIOD_S);
            if (optSamplePoint.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TrajectorySamplePoint samplePoint = optSamplePoint.get();

            TimedPose desiredState = samplePoint.state();
            SwerveModel reference = SwerveModel.fromTimedPose(desiredState);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(measurement, reference);

            m_steeringAligned = m_swerve.steerAtRest(fieldRelativeTarget);
        }
    }

    @Override
    public boolean isFinished() {
        if (m_trajectory == null)
            return true;
        return m_iter.isDone() && m_controller.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

}
