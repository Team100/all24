package org.team100.lib.commands;

import java.util.function.BiFunction;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.sway.controller.HolonomicDriveRegulator;
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
    private final SwerveDriveSubsystem m_swerve;
    private final Timer m_timer;
    private final HolonomicDriveController3 m_controller;
    private final BiFunction<Pose2d, Pose2d, Trajectory> m_trajectories;

    private Trajectory m_trajectory;

    public DriveToWaypoint3(
            Pose2d goal,
            SwerveDriveSubsystem drivetrain,
            BiFunction<Pose2d, Pose2d, Trajectory> trajectories) {
        m_goal = goal;
        m_swerve = drivetrain;
        m_trajectories = trajectories;
        m_timer = new Timer();
        Identity identity = Identity.get();

        DriveControllers controllers = new DriveControllersFactory().get(identity);

        m_controller = new HolonomicDriveController3(controllers);
        m_controller.setTolerance(0.1, 1.0);

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_trajectory = m_trajectories.apply(m_swerve.getPose(), m_goal);
        System.out.println(m_trajectory);
        m_timer.restart();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        if (m_trajectory == null)
            return;
        State desiredState = m_trajectory.sample(curTime);
        Pose2d currentPose = m_swerve.getPose();
        // TODO: rotation profile, use new trajectory type.
        SwerveState reference = SwerveState.fromState(desiredState, m_goal.getRotation());
        Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);

        m_swerve.driveInFieldCoords(fieldRelativeTarget);
        t.log(Level.DEBUG, "/Drive To Waypoint/Desired X", desiredState.poseMeters.getX());
        t.log(Level.DEBUG, "/Drive To Waypoint/Desired Y", desiredState.poseMeters.getY());
        t.log(Level.DEBUG, "/Drive To Waypoint/Pose X", m_swerve.getPose().getX());
        t.log(Level.DEBUG, "/Drive To Waypoint/Pose Y", m_swerve.getPose().getY());
        t.log(Level.DEBUG, "/Drive To Waypoint/Desired Rot", m_goal.getRotation().getRadians());
        t.log(Level.DEBUG, "/Drive To Waypoint/Pose Rot", m_swerve.getPose().getRotation().getRadians());
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
