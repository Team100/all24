package org.team100.frc2024.motion.drivetrain;

import java.util.List;
import java.util.Optional;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimingConstraint;
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

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 */
public class DriveToAdjacentWithShooterAngle extends Command100 {
    // inject these, make them the same as the kinematic limits, inside the
    // trajectory supplier.
    private static final double kMaxVelM_S = 4;
    private static final double kMaxAccelM_S_S = 4;
    private static final Telemetry t = Telemetry.get();

    private final SwerveDriveSubsystem m_swerve;
    private final Translation2d m_goalTranslation;
    private final TrajectoryPlanner m_planner;
    private final DriveMotionController m_controller;
    private final List<TimingConstraint> m_constraints;
    private final double kShooterScale;

    public DriveToAdjacentWithShooterAngle(
            SwerveDriveSubsystem swerve,
            Translation2d goalTranslation,
            TrajectoryPlanner planner,
            DriveMotionController controller,
            List<TimingConstraint> constraints,
            double shooterScale) {
        m_swerve = swerve;
        m_goalTranslation = goalTranslation;
        m_planner = planner;
        m_controller = controller;
        m_constraints = constraints;
        kShooterScale = shooterScale;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
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
        // Translation2d offset = new Translation2d();
        Pose2d endWaypoint = new Pose2d(m_goalTranslation.plus(offset), endRotation);
        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(ShooterUtil.getRobotRotationToSpeaker(optionalAlliance.get(),
                startPose.getTranslation(), kShooterScale),
                endHeading);
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(trajectory));
        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute100(double dt) {
        double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_swerve.getPose();
        ChassisSpeeds currentSpeed = m_swerve.speeds(dt);
        ChassisSpeeds output = m_controller.update(now, currentPose, currentSpeed);

        t.log(Level.DEBUG, m_name, "chassis speeds", output);
        DriveUtil.checkSpeeds(output);
        m_swerve.setChassisSpeeds(output, dt);
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        TrajectoryVisualization.clear();
    }
}
