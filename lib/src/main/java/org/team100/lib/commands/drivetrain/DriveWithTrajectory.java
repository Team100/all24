package org.team100.lib.commands.drivetrain;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.ChassisSpeedsLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry.Level;
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

public class DriveWithTrajectory extends Command implements Glassy  {
    private static final double kMaxVel = 5;
    private static final double kMaxAcc = 5;
    private static final double kStartVel = 0;
    private static final double kEndVel = 0;

    private final SwerveDriveSubsystem m_swerve;
    private final DriveTrajectoryFollower m_controller;
    private final Trajectory100 trajectory;
    private final TrajectoryVisualization m_viz;

    // LOGGERS
    private final ChassisSpeedsLogger m_log_chassis_speeds;

    public DriveWithTrajectory(
            SupplierLogger2 parent,
            SwerveDriveSubsystem drivetrain,
            DriveTrajectoryFollower controller,
            SwerveKinodynamics limits,
            String fileName,
            TrajectoryVisualization viz) {
        SupplierLogger2 child = parent.child(this);
        m_log_chassis_speeds = child.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
        m_swerve = drivetrain;
        m_controller = controller;

        PathArrays trajectoryList = JSONParser.getTrajectoryList(fileName);
        trajectoryList.removeLastIndex();

        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).allGood();
        List<Pose2d> poses = getWaypoints(trajectoryList.getPoseArray());
        List<Rotation2d> headings = trajectoryList.getRotationArray();

        trajectory = TrajectoryPlanner.generateTrajectory(
                poses,
                headings,
                constraints,
                kStartVel,
                kEndVel,
                kMaxVel,
                kMaxAcc);
        m_viz = viz;

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_viz.setViz(trajectory);
        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(trajectory));
        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_swerve.getState().pose();
        ChassisSpeeds currentSpeed = m_swerve.getState().chassisSpeeds();
        ChassisSpeeds output = m_controller.update(now, currentPose, currentSpeed);

        m_log_chassis_speeds.log(() -> output);
        DriveUtil.checkSpeeds(output);
        m_swerve.setChassisSpeeds(output);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        m_viz.clear();
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    private static List<Pose2d> getWaypoints(List<Pose2d> m) {
        List<Pose2d> waypointsM = new ArrayList<>();
        for (int i = 0; i < m.size() - 1; i += 1) {
            Translation2d t0 = m.get(i).getTranslation();
            Translation2d t1 = m.get(i + 1).getTranslation();
            Rotation2d theta = t1.minus(t0).getAngle();
            waypointsM.add(new Pose2d(t0, theta));
        }
        // Last Value
        Translation2d t0 = m.get(m.size() - 1).getTranslation();
        Translation2d t1 = m.get(m.size() - 2).getTranslation();
        Rotation2d theta = t1.minus(t0).getAngle();
        waypointsM.add(new Pose2d(t0, theta));
        return waypointsM;
    }
}
