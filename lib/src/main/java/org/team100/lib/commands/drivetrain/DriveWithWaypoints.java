package org.team100.lib.commands.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class DriveWithWaypoints extends Command100 {
    private static final Telemetry t = Telemetry.get();

    private static final double max_vel = 5;
    private static final double max_acc = 5;
    private static final double start_vel = 0;
    private static final double end_vel = 0;

    private final SwerveDriveSubsystem m_swerve;
    private final TrajectoryPlanner m_planner;
    private final DriveMotionController m_controller;
    private final List<TimingConstraint> constraints;
    private final Supplier<List<Pose2d>> m_goal;

    public DriveWithWaypoints(SwerveDriveSubsystem drivetrain,
            TrajectoryPlanner planner,
            DriveMotionController controller,
            SwerveKinodynamics limits,
            Supplier<List<Pose2d>> goal) {
        m_swerve = drivetrain;
        m_planner = planner;
        m_controller = controller;
        constraints = new TimingConstraintFactory(limits).allGood();
        m_goal = goal;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        final Pose2d start = m_swerve.getPose();
        List<Pose2d> newWaypointM = new ArrayList<>(m_goal.get());
        newWaypointM.add(0, start);

        List<Rotation2d> headings = new ArrayList<>();
        for (int i = 0; i < newWaypointM.size(); i++) {
            headings.add(newWaypointM.get(i).getRotation());
        }

        newWaypointM = getWaypointsList(newWaypointM);

        Trajectory100 trajectory = m_planner
                .generateTrajectory(
                        false,
                        newWaypointM,
                        headings,
                        constraints,
                        start_vel,
                        end_vel,
                        max_vel,
                        max_acc);

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
    public void end100(boolean interrupted) {
        m_swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return m_controller.isDone();
    }

    private static List<Pose2d> getWaypointsList(List<Pose2d> m) {
        List<Pose2d> waypointsM = new ArrayList<>();
        for (int i = 0; i < m.size() - 1; i += 1) {
            Translation2d t0 = m.get(i).getTranslation();
            Translation2d t1 = m.get(i + 1).getTranslation();
            Rotation2d theta = t1.minus(t0).getAngle();
            waypointsM.add(new Pose2d(t0, theta));
        }

        Translation2d t0 = m.get(m.size() - 1).getTranslation();
        Translation2d t1 = m.get(m.size() - 2).getTranslation();
        Rotation2d theta = t0.minus(t1).getAngle();
        waypointsM.add(new Pose2d(t0, theta));
        return waypointsM;
    }
}
