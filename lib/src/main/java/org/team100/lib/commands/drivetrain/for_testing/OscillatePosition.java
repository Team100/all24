package org.team100.lib.commands.drivetrain.for_testing;

import java.util.Optional;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.util.Util;


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

    private Trajectory100 m_trajectory;
    private TrajectoryTimeIterator m_iter;

    private boolean m_steeringAligned;

    public OscillatePosition(
            SwerveDriveSubsystem drivetrain,
            StraightLineTrajectory trajectories,
            HolonomicFieldRelativeController controller,
            double offsetM) {
        m_swerve = drivetrain;
        m_trajectories = trajectories;
        m_controller = controller;
        m_offsetM = offsetM;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_controller.reset();
        // choose a goal 1m away
        SwerveState start = m_swerve.getState();
        Pose2d startPose = start.pose();
        Pose2d endPose = startPose.plus(new Transform2d(m_offsetM, 0, new Rotation2d()));
        m_trajectory = m_trajectories.apply(start, endPose);
        m_iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(m_trajectory));
        m_steeringAligned = false;
    }

    @Override
    public void execute() {
        if (m_trajectory == null)
            return;
        SwerveState measurement = m_swerve.getState();

        if (m_steeringAligned) {
            Optional<TrajectorySamplePoint> optSamplePoint = m_iter.advance(TimedRobot100.LOOP_PERIOD_S);
            if (optSamplePoint.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TrajectorySamplePoint samplePoint = optSamplePoint.get();

            TimedPose desiredState = samplePoint.state();
            SwerveState reference = SwerveState.fromTimedPose(desiredState);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(measurement, reference);

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
            SwerveState reference = SwerveState.fromTimedPose(desiredState);
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
