package org.team100.frc2024.motion;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.BooleanSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.ChassisSpeedsLogger;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.Pose2dLogger;
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
    private final SwerveDriveSubsystem m_robotDrive;
    private final DriveMotionController m_controller;
    private final Trajectory100 m_trajectory;
    private final Pose2d m_goal;
    private final TrajectoryVisualization m_viz;

    // LOGGERS
    private final Pose2dLogger m_log_goal;
    private final ChassisSpeedsLogger m_log_chassis_speeds;
    private final DoubleSupplierLogger2 m_log_THETA_ERROR;
    private final BooleanSupplierLogger2 m_log_FINSIHED;

    public TrajectoryCommand100(
            SupplierLogger2 parent,
            SwerveDriveSubsystem robotDrive,
            Trajectory100 trajectory,
            DriveMotionController controller,
            TrajectoryVisualization viz) {
        super(parent);
        m_log_goal = m_logger.pose2dLogger(Level.TRACE, "goal");
        m_log_chassis_speeds = m_logger.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
        m_log_THETA_ERROR = m_logger.doubleLogger(Level.TRACE, "THETA ERROR");
        m_log_FINSIHED = m_logger.booleanLogger(Level.TRACE, "FINSIHED");
        m_robotDrive = robotDrive;
        m_trajectory = trajectory;
        m_controller = controller;
        m_goal = m_trajectory.getLastPoint().state().state().getPose();
        m_viz = viz;
        m_log_goal.log(() -> m_goal);
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize100() {
        m_viz.setViz(m_trajectory);
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

        m_log_chassis_speeds.log(() -> output);
        double thetaErrorRad = m_goal.getRotation().getRadians()
                - m_robotDrive.getState().pose().getRotation().getRadians();
        m_log_THETA_ERROR.log(() -> thetaErrorRad);
        m_log_FINSIHED.log(() -> false);
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    @Override
    public void end100(boolean interrupted) {
        m_log_FINSIHED.log(() -> true);
        m_robotDrive.stop();
        m_viz.clear();
    }
}
