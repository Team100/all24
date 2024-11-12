package org.team100.frc2024.motion;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follow a fixed trajectory, using the new 254-derived trajectory and follower
 * types.
 * 
 * This is an experiment.
 */
public class TrajectoryCommand100 extends Command implements Glassy  {
    /**
     * Log exists so multiple commands can use the same keys.
     */
    public static class Log {
        private final Pose2dLogger m_log_goal;
        private final ChassisSpeedsLogger m_log_chassis_speeds;
        private final DoubleLogger m_log_THETA_ERROR;
        private final BooleanLogger m_log_FINSIHED;

        public Log(LoggerFactory parent) {
            LoggerFactory log = parent.child("TrajectoryCommand100");
            m_log_goal = log.pose2dLogger(Level.TRACE, "goal");
            m_log_chassis_speeds = log.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
            m_log_THETA_ERROR = log.doubleLogger(Level.TRACE, "THETA ERROR");
            m_log_FINSIHED = log.booleanLogger(Level.TRACE, "FINSIHED");
        }
    }

    private final Log m_log;
    private final SwerveDriveSubsystem m_robotDrive;
    private final DriveTrajectoryFollower m_controller;
    private final Trajectory100 m_trajectory;
    private final Pose2d m_goal;
    private final TrajectoryVisualization m_viz;

    public TrajectoryCommand100(
            Log log,
            SwerveDriveSubsystem robotDrive,
            Trajectory100 trajectory,
            DriveTrajectoryFollower controller,
            TrajectoryVisualization viz) {
        m_log = log;
        m_robotDrive = robotDrive;
        m_trajectory = trajectory;
        m_controller = controller;
        m_goal = m_trajectory.getLastPoint().state().state().getPose();
        m_viz = viz;
        log.m_log_goal.log(() -> m_goal);
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        m_viz.setViz(m_trajectory);
        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(m_trajectory));
        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute() {
        final double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_robotDrive.getPose();
        ChassisSpeeds currentRobotRelativeSpeed = m_robotDrive.getChassisSpeeds();
        ChassisSpeeds output = m_controller.update(now, currentPose, currentRobotRelativeSpeed);

        m_robotDrive.setChassisSpeedsNormally(output);

        m_log.m_log_chassis_speeds.log(() -> output);
        double thetaErrorRad = m_goal.getRotation().getRadians()
                - m_robotDrive.getPose().getRotation().getRadians();
        m_log.m_log_THETA_ERROR.log(() -> thetaErrorRad);
        m_log.m_log_FINSIHED.log(() -> false);
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_log.m_log_FINSIHED.log(() -> true);
        m_robotDrive.stop();
        m_viz.clear();
    }
}
