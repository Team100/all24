package org.team100.frc2024.motion.drivetrain;

import java.util.List;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 */
public class DriveToWithAutoStart extends Command implements Glassy {
    private final SwerveDriveSubsystem m_swerve;
    private final Pose2d m_goalWaypoint;
    private final Rotation2d m_goalHeading;
    private final DriveTrajectoryFollower m_controller;
    private final List<TimingConstraint> m_constraints;
    private final TrajectoryVisualization m_viz;

    // LOGGERS
    private final ChassisSpeedsLogger m_log_chassis_speeds;

    public DriveToWithAutoStart(
            LoggerFactory parent,
            SwerveDriveSubsystem swerve,
            Pose2d goalWaypoint,
            Rotation2d goalHeading,
            DriveTrajectoryFollower controller,
            SwerveKinodynamics swerveKinodynamics,
            TrajectoryVisualization viz) {
        LoggerFactory child = parent.child(this);
        m_log_chassis_speeds = child.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
        m_swerve = swerve;
        m_goalWaypoint = goalWaypoint;
        m_goalHeading = goalHeading;
        m_controller = controller;
        m_constraints = new TimingConstraintFactory(swerveKinodynamics).fast();
        m_viz = viz;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        Pose2d startPose = m_swerve.getPose();
        Translation2d startTranslation = new Translation2d();
        Translation2d endTranslation = m_goalWaypoint.getTranslation();
        Rotation2d angleToGoal = endTranslation.minus(startTranslation).getAngle();
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), angleToGoal);

        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                m_goalWaypoint);
        List<Rotation2d> headings = List.of(
                startPose.getRotation(),
                m_goalHeading);
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_constraints);

        if (trajectory.length() == 0) {
            end(false);
            return;
        }

        m_viz.setViz(trajectory);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(trajectory));

        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_swerve.getPose();
        ChassisSpeeds currentSpeed = m_swerve.getChassisSpeeds();
        ChassisSpeeds output = m_controller.update(now, currentPose, currentSpeed);

        m_log_chassis_speeds.log(() -> output);
        DriveUtil.checkSpeeds(output);
        m_swerve.setChassisSpeedsNormally(output);
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        m_viz.clear();
    }
}
