package org.team100.lib.commands.drivetrain;

import java.util.function.BiFunction;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToWaypoint3 extends Command {
    private final Telemetry t = Telemetry.get();
    private final Pose2d m_goal;
    private final SwerveDriveSubsystemInterface m_swerve;
    private final Timer m_timer;
    private final HolonomicDriveController3 m_controller;
    private final BiFunction<SwerveState, Pose2d, Trajectory> m_trajectories;

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
            SwerveDriveSubsystemInterface drivetrain,
            BiFunction<SwerveState, Pose2d, Trajectory> trajectories,
            HolonomicDriveController3 controller) {
        m_goal = goal;
        m_swerve = drivetrain;
        m_trajectories = trajectories;
        m_controller = controller;
        m_timer = new Timer();
        if (m_swerve.get() != null)
            addRequirements(m_swerve.get());
    }

    @Override
    public void initialize() {
        m_trajectory = m_trajectories.apply(m_swerve.getState(), m_goal);
        System.out.println(m_trajectory);
        m_timer.stop();
        m_timer.reset();
        m_steeringAligned = false;
    }

    @Override
    public void execute() {
        if (m_trajectory == null)
            return;

        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);
        Pose2d currentPose = m_swerve.getPose();
        // TODO: rotation profile, use new trajectory type.
        SwerveState reference = SwerveState.fromState(desiredState, m_goal.getRotation());
        Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);

        if (m_steeringAligned) {
            // follow normally
            // System.out.println("aligned");
            m_swerve.driveInFieldCoords(fieldRelativeTarget);
        } else {
            // not aligned yet, try aligning
            // System.out.println("aligning...");

            boolean aligned = m_swerve.steerAtRest(fieldRelativeTarget);
            if (aligned) {
                m_steeringAligned = true;
                m_timer.start();
                m_swerve.driveInFieldCoords(fieldRelativeTarget);
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
    }
}
