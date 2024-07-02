package org.team100.frc2024.motion;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private final Trajectory100 m_trajectory;
    private final Pose2d m_goal;

    public TrajectoryCommand100(
            SwerveDriveSubsystem robotDrive,
            Trajectory100 trajectory,
            DriveMotionController controller) {
        m_robotDrive = robotDrive;
        m_trajectory = trajectory;
        m_controller = controller;
        m_goal = m_trajectory.getLastPoint().state().state().getPose();
        t.log(Level.TRACE, m_name, "goal", m_goal);
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
        Pose2d currentPose = m_robotDrive.getState().pose();
        ChassisSpeeds currentRobotRelativeSpeed = m_robotDrive.getState().chassisSpeeds();
        ChassisSpeeds output = m_controller.update(now, currentPose, currentRobotRelativeSpeed);

        m_robotDrive.setChassisSpeedsNormally(output, dt);

        t.log(Level.TRACE, m_name, "chassis speeds", output);
        double thetaErrorRad = m_goal.getRotation().getRadians() - m_robotDrive.getState().pose().getRotation().getRadians();
        t.log(Level.TRACE, m_name, "THETA ERROR", thetaErrorRad);
        t.log(Level.TRACE, m_name, "FINSIHED", false);
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    @Override
    public void end100(boolean interrupted) {
        t.log(Level.TRACE, m_name, "FINSIHED", true);
        m_robotDrive.stop();
        TrajectoryVisualization.clear();
    }
}
