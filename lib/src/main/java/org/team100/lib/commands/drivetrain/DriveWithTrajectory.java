package org.team100.lib.commands.drivetrain;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
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

public class DriveWithTrajectory extends Command100 {
    private static final Telemetry t = Telemetry.get();
    private static final double kMaxVel = 5;
    private static final double kMaxAcc = 5;
    private static final double kStartVel = 0;
    private static final double kEndVel = 0;
    private final SwerveDriveSubsystem m_swerve;
    private final DriveMotionController m_controller;
    private final Trajectory100 trajectory;

    public DriveWithTrajectory(SwerveDriveSubsystem drivetrain,
            TrajectoryPlanner planner,
            DriveMotionController controller,
            SwerveKinodynamics limits,
            String fileName) {
        m_swerve = drivetrain;
        m_controller = controller;

        PathArrays trajectoryList = JSONParser.getTrajectoryList(fileName);
        trajectoryList.removeLastIndex();

        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).allGood();
        List<Pose2d> poses = getWaypoints(trajectoryList.getPoseArray());
        List<Rotation2d> headings = trajectoryList.getRotationArray();

        trajectory = planner
                .generateTrajectory(
                        false,
                        poses,
                        headings,
                        constraints,
                        kStartVel,
                        kEndVel,
                        kMaxVel,
                        kMaxAcc);

        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        TrajectoryVisualization.setViz(trajectory);
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
    public void end(boolean interrupted) {
        m_swerve.stop();
        TrajectoryVisualization.clear();
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
