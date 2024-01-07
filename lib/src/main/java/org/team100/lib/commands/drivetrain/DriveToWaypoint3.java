package org.team100.lib.commands.drivetrain;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;

/**
 * Drive from the current state to a field-relative goal.
 * 
 * The trajectory is supplied; the supplier is free to ignore the current state.
 * 
 * The goal rotation is used as the setpoint the entire time, which will put
 * a lot of error into the rotational controller.
 * 
 * If you want a holonomic trajectory follower, try the
 * {@link DriveMotionController} classes.
 */
public class DriveToWaypoint3 extends Command100 {
    private final Telemetry t = Telemetry.get();
    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_swerve;
    private final Timer m_timer;
    private final HolonomicDriveController3 m_controller;
    private final StraightLineTrajectory m_trajectories;

    private Trajectory m_trajectory;
    /**
     * Trajectory waits until wheels are aligned. If we depend on the setpoint
     * generator to do it, then we're behind the profile timer. After the initial
     * alignment, the steering should be able to keep up with the profile.
     */
    private boolean m_steeringAligned;

    /**
     * @param trajectories function that takes a start and end pose and returns a
     *                     trajectory between them.
     */
    public DriveToWaypoint3(
            Pose2d goal,
            SwerveDriveSubsystem drivetrain,
            StraightLineTrajectory trajectories,
            HolonomicDriveController3 controller) {
        m_goal = goal;
        m_swerve = drivetrain;
        m_trajectories = trajectories;
        m_controller = controller;
        m_timer = new Timer();
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        m_controller.reset();
        m_trajectory = m_trajectories.apply(m_swerve.getState(0.02), m_goal);
        TrajectoryVisualization.setViz(m_trajectory);
        m_timer.stop();
        m_timer.reset();
        m_steeringAligned = false;
    }

    @Override
    public void execute100(double dt) {
        if (m_trajectory == null)
            return;

        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);
        Pose2d currentPose = m_swerve.getPose();
        SwerveState reference = SwerveState.fromState(desiredState, m_goal.getRotation());
        Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);

        if (m_steeringAligned) {
            // follow normally
            m_swerve.driveInFieldCoords(fieldRelativeTarget, dt);
        } else {
            // not aligned yet, try aligning
            boolean aligned = m_swerve.steerAtRest(fieldRelativeTarget, dt);
            if (aligned) {
                m_steeringAligned = true;
                m_timer.start();
                m_swerve.driveInFieldCoords(fieldRelativeTarget, dt);
            }
        }

        t.log(Level.DEBUG, "/Drive To Waypoint/Aligned", m_steeringAligned);
        t.log(Level.DEBUG, "/Drive To Waypoint/Desired X", desiredState.poseMeters.getX());
        t.log(Level.DEBUG, "/Drive To Waypoint/Desired Y", desiredState.poseMeters.getY());
        t.log(Level.DEBUG, "/Drive To Waypoint/Pose X", m_swerve.getPose().getX());
        t.log(Level.DEBUG, "/Drive To Waypoint/Pose Y", m_swerve.getPose().getY());
        t.log(Level.DEBUG, "/Drive To Waypoint/Desired Rot", m_goal.getRotation().getRadians());
        t.log(Level.DEBUG, "/Drive To Waypoint/Pose Rot", m_swerve.getPose().getRotation().getRadians());
        t.log(Level.DEBUG, "/Drive To Waypoint/Time", curTime);
    }

    @Override
    public boolean isFinished() {
        if (m_trajectory == null)
            return true;
        return m_timer.get() > m_trajectory.getTotalTimeSeconds() && m_controller.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        TrajectoryVisualization.clear();
    }
}
