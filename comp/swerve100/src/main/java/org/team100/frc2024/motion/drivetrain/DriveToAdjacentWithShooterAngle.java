package org.team100.frc2024.motion.drivetrain;

import java.util.List;
import java.util.Optional;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 */
public class DriveToAdjacentWithShooterAngle extends Command implements Glassy {
    private final SwerveDriveSubsystem m_swerve;
    private final Translation2d m_goalTranslation;
    private final DriveTrajectoryFollower m_controller;
    private final List<TimingConstraint> m_constraints;
    private final double kShooterScale;
    private final TrajectoryVisualization m_viz;

    // LOGGERS
    private final ChassisSpeedsLogger m_log_chassis_speeds;

    public DriveToAdjacentWithShooterAngle(
            LoggerFactory parent,
            SwerveDriveSubsystem swerve,
            Translation2d goalTranslation,
            DriveTrajectoryFollower controller,
            SwerveKinodynamics swerveKinodynamics,
            double shooterScale,
            TrajectoryVisualization viz) {
        LoggerFactory child = parent.child(this);
        m_log_chassis_speeds = child.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
        m_swerve = swerve;
        m_goalTranslation = goalTranslation;
        m_controller = controller;
        m_constraints = new TimingConstraintFactory(swerveKinodynamics).fast();
        kShooterScale = shooterScale;
        m_viz = viz;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (!optionalAlliance.isPresent())
            return;
        Pose2d startPose = m_swerve.getPose();
        Rotation2d rotationToGoal = m_goalTranslation.minus(startPose.getTranslation()).getAngle();
        Rotation2d startRotation = rotationToGoal.times(1.5);
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), startRotation);
        Rotation2d endHeading = ShooterUtil.getRobotRotationToSpeaker(optionalAlliance.get(),
                m_goalTranslation, kShooterScale);
        Rotation2d endRotation = endHeading.plus(new Rotation2d(Math.PI));
        Translation2d offset = new Translation2d(-.5 * endRotation.getCos(), -.5 *
                endRotation.getSin());
        Pose2d endWaypoint = new Pose2d(m_goalTranslation.plus(offset), endRotation);
        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(
                ShooterUtil.getRobotRotationToSpeaker(
                        optionalAlliance.get(), startPose.getTranslation(), kShooterScale),
                endHeading);
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_constraints);
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
        m_swerve.setChassisSpeeds(output);
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
