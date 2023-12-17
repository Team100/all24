package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 */
public class DriveToWaypoint100 extends Command {
    // inject these, make them the same as the kinematic limits, inside the
    // trajectory supplier.
    private static final double kMaxVelM_S = 4;
    private static final double kMaxAccelM_S_S = 2;
    private static final double kMaxVoltage = 9.0;

    private final Telemetry t = Telemetry.get();
    private final Pose2d m_goal;
    private final SwerveDriveSubsystemInterface m_swerve;
    private final TrajectoryPlanner m_planner;
    private final DriveMotionController m_controller;

    public DriveToWaypoint100(
            Pose2d goal,
            SwerveDriveSubsystemInterface drivetrain,
            TrajectoryPlanner planner,
            DriveMotionController controller) {
        m_goal = goal;
        m_swerve = drivetrain;
        m_planner = planner;
        m_controller = controller;
        if (m_swerve.get() != null)
            addRequirements(m_swerve.get());
    }

    @Override
    public void initialize() {
        Pose2d start = m_swerve.getPose();
        double startVelocity = 0;
        Pose2d end = m_goal;
        double endVelocity = 0;

        // TODO: put this angle calculation in a class like StraightLineTrajectory
        Translation2d currentTranslation = start.getTranslation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        List<Pose2d> waypointsM = List.of(
                new Pose2d(currentTranslation, angleToGoal),
                new Pose2d(goalTranslation, angleToGoal));

        List<Rotation2d> headings = List.of(
                start.getRotation(),
                end.getRotation());

        List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        Trajectory100 trajectory = m_planner
                .generateTrajectory(
                        false,
                        waypointsM,
                        headings,
                        constraints,
                        startVelocity,
                        endVelocity,
                        kMaxVelM_S,
                        kMaxAccelM_S_S,
                        kMaxVoltage);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(trajectory));

        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_swerve.getPose();
        ChassisSpeeds currentSpeed = m_swerve.speeds();
        Twist2d velocity = new Twist2d(
                currentSpeed.vxMetersPerSecond,
                currentSpeed.vyMetersPerSecond,
                currentSpeed.omegaRadiansPerSecond);
        ChassisSpeeds output = m_controller.update(now, currentPose, velocity);
        t.log(Level.DEBUG, "/fancy trajectory/chassis speeds", output);
        if (Double.isNaN(output.vxMetersPerSecond))
            throw new IllegalStateException("vx is NaN");
        if (Double.isNaN(output.vyMetersPerSecond))
            throw new IllegalStateException("vy is NaN");
        if (Double.isNaN(output.omegaRadiansPerSecond))
            throw new IllegalStateException("omega is NaN");
        m_swerve.setChassisSpeeds(output);
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

}
