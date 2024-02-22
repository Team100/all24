package org.team100.frc2024.motion;

import java.util.List;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.trajectory.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.Timer;

/**
 * Follow a fixed trajectory, using the new 254-derived trajectory and follower
 * types.
 * 
 * This is an experiment.
 */
public class TrajectoryCommand100 extends Command100 {
    private final Telemetry t = Telemetry.get();

    private final SwerveDriveSubsystem m_robotDrive;
    private final DriveMotionController m_controller;
    private Trajectory100 m_trajectory;

    public TrajectoryCommand100(
            SwerveDriveSubsystem robotDrive,
            Trajectory100 trajectory,
            DriveMotionController controller) {
        m_robotDrive = robotDrive;
        m_trajectory = trajectory;
        m_controller = controller;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize100() {
        TrajectoryVisualization.setViz(m_trajectory);
        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(m_trajectory));
        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute100(double dt) {
        final double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_robotDrive.getPose();
        ChassisSpeeds currentSpeed = m_robotDrive.speeds(dt);
        Twist2d velocity = GeometryUtil.toTwist2d(currentSpeed);
        ChassisSpeeds output = m_controller.update(now, currentPose, velocity);
        t.log(Level.TRACE, m_name, "chassis speeds", output);
        m_robotDrive.setChassisSpeeds(output, dt);
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        TrajectoryVisualization.clear();
    }
}
