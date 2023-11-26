package org.team100.lib.commands.drivetrain;

import java.util.function.BiFunction;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.PidGains;
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

/** TODO: i think this is the same as DriveToWaypoint3? */
public class DriveToAprilTag extends Command {
    private final Telemetry t = Telemetry.get();
    private final Pose2d m_goal;
    private final SwerveDriveSubsystemInterface m_swerve;
    private final Timer m_timer;
    private final HolonomicDriveController3 m_controller;
    private final BiFunction<Pose2d, Pose2d, Trajectory> m_trajectories;

    private Trajectory m_trajectory;

    public DriveToAprilTag(
            Pose2d goal,
            SwerveDriveSubsystemInterface drivetrain,
            BiFunction<Pose2d, Pose2d, Trajectory> trajectories) {
        m_goal = goal;
        m_swerve = drivetrain;
        m_timer = new Timer();
        m_trajectories = trajectories;

        Identity identity = Identity.get();

        DriveControllers controllers = new DriveControllersFactory().get(identity);

        m_controller = new HolonomicDriveController3(controllers);
        m_controller.setTolerance(0.1, 1.0);
        m_controller.setGains(
                new PidGains(2, 0, 0, 0, 0.01, false),
                new PidGains(6.5, 0, 1, 0, 0.01, true));
        m_controller.setIRange(0.3);
        m_controller.setTolerance(0.00000001, Math.PI / 180);

        if (drivetrain.get() != null)
            addRequirements(drivetrain.get());
    }

    @Override
    public void initialize() {
        m_trajectory = m_trajectories.apply(m_swerve.getPose(), m_goal);
        m_timer.restart();
    }

    @Override
    public void execute() {
        if (m_trajectory == null) {
            return;
        }
        State desiredState = m_trajectory.sample(m_timer.get());
        Pose2d currentPose = m_swerve.getPose();
        // TODO: rotation profile, use new trajectory type
        SwerveState reference = SwerveState.fromState(desiredState, m_goal.getRotation());
        Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);
        m_swerve.driveInFieldCoords(fieldRelativeTarget);
        t.log(Level.DEBUG, "/desired pose/x", reference.x().x());
        t.log(Level.DEBUG, "/desired pose/y", reference.y().x());
        t.log(Level.DEBUG, "/desired pose/theta", reference.theta().x());
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
