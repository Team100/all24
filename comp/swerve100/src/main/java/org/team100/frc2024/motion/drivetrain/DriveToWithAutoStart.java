package org.team100.frc2024.motion.drivetrain;

import java.util.List;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.telemetry.Telemetry.Logger;
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
import edu.wpi.first.wpilibj.Timer;

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 */
public class DriveToWithAutoStart extends Command100 {

    private static final double kMaxVelM_S = 4;
    private static final double kMaxAccelM_S_S = 5;

    private final SwerveDriveSubsystem m_swerve;
    private final Pose2d m_goalWaypoint;
    private final Rotation2d m_goalHeading;
    private final DriveMotionController m_controller;
    private final List<TimingConstraint> m_constraints;

    public DriveToWithAutoStart(
            Logger parent,
            SwerveDriveSubsystem swerve,
            Pose2d goalWaypoint,
            Rotation2d goalHeading,
            DriveMotionController controller,
            List<TimingConstraint> constraints) {
        super(parent);
        m_swerve = swerve;
        m_goalWaypoint = goalWaypoint;
        m_goalHeading = goalHeading;
        m_controller = controller;
        m_constraints = constraints;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        Pose2d startPose = m_swerve.getState().pose();
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
        Trajectory100 trajectory = TrajectoryPlanner.generateTrajectory(
                waypointsM,
                headings,
                m_constraints,
                0.0,
                0.0,
                kMaxVelM_S,
                kMaxAccelM_S_S);

        if (trajectory.length() == 0) {
            end(false);
            return;
        }

        // TrajectoryVisualization.setViz(trajectory);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(trajectory));

        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute100(double dt) {
        double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_swerve.getState().pose();
        ChassisSpeeds currentSpeed = m_swerve.getState().chassisSpeeds();
        ChassisSpeeds output = m_controller.update(now, currentPose, currentSpeed);

        m_logger.log(Level.DEBUG, "chassis speeds", output);
        DriveUtil.checkSpeeds(output);
        m_swerve.setChassisSpeedsNormally(output, dt);
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    @Override
    public void end100(boolean interrupted) {
        m_swerve.stop();
        TrajectoryVisualization.clear();
    }
}
