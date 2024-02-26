package org.team100.lib.commands.drivetrain;

import java.util.List;
import java.util.function.Supplier;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
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
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 */
public class DriveToState101 extends Command100 {
    // inject these, make them the same as the kinematic limits, inside the
    // trajectory supplier.
    private static final double kMaxVelM_S = 4;
    private static final double kMaxAccelM_S_S = 4;
    private static final Telemetry t = Telemetry.get();

    private final Pose2d m_goal;
    private final Twist2d m_endVelocity;
    private final SwerveDriveSubsystem m_swerve;
    private final TrajectoryPlanner m_planner;
    private final DriveMotionController m_controller;
    private final SwerveKinodynamics m_limits;

    /**
     * @param goal        Pose2d
     * @param endVelocity Twist2d
     * @param drivetrain  SwerveDriveSubsystem
     * @param planner     TrajectoryPlanner
     * @param controller  DriveMotionController
     * @param limits      SwerveKinodynamics
     * @param viz         ok to be null
     */

    public DriveToState101(
            Pose2d goal,
            Twist2d endVelocity,
            SwerveDriveSubsystem drivetrain,
            TrajectoryPlanner planner,
            DriveMotionController controller,
            SwerveKinodynamics limits) {
        m_goal = goal;
        m_endVelocity = endVelocity;
        m_swerve = drivetrain;
        m_planner = planner;
        m_controller = controller;
        m_limits = limits;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        System.out.println("DRIVE TO STATE");


        Transform2d transform = new Transform2d(m_goal.getTranslation().minus(m_swerve.getPose().getTranslation()), m_goal.getTranslation().minus(m_swerve.getPose().getTranslation()).getAngle());

        transform = transform.inverse();

        Pose2d startPose = new Pose2d(m_swerve.getPose().getTranslation(), transform.getRotation());

        Twist2d startVelocity = m_swerve.getVelocity();

        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(),
                new Rotation2d(1, 1));

        if(startVelocity.dx == 0 && startVelocity.dy == 0){
            startWaypoint = startPose;
        } else {
            startWaypoint = new Pose2d(startPose.getTranslation(), new Rotation2d(startVelocity.dx, startVelocity.dy));

        }        


        Pose2d endWaypoint = new Pose2d(m_goal.getTranslation(),
                new Rotation2d(1, -1));

        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                endWaypoint);
        List<Rotation2d> headings = List.of(
                m_swerve.getPose().getRotation(),
                m_goal.getRotation());

        List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(m_limits));

        Trajectory100 trajectory = m_planner
                .generateTrajectory(
                        false,
                        waypointsM,
                        headings,
                        constraints,
                        Math.hypot(startVelocity.dx, startVelocity.dy),
                        Math.hypot(m_endVelocity.dx, m_endVelocity.dy),
                        kMaxVelM_S,
                        kMaxAccelM_S_S);

        if (trajectory.length() == 0) {
            end(false);
            return;
        }

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
        Twist2d velocity = new Twist2d(
                currentSpeed.vxMetersPerSecond,
                currentSpeed.vyMetersPerSecond,
                currentSpeed.omegaRadiansPerSecond);
        ChassisSpeeds output = m_controller.update(now, currentPose, velocity);

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
        System.out.println("FINNIISHEDD");
        m_swerve.stop();
        TrajectoryVisualization.clear();
    }

    ////////////////////////////////////////////////////

    /** Waypoints where the rotation points in the direction of motion. */
    private static List<Pose2d> getWaypoints(Pose2d p0, Pose2d p1) {
        Translation2d t0 = p0.getTranslation();
        Translation2d t1 = p1.getTranslation();
        Rotation2d theta = t1.minus(t0).getAngle();
        return List.of(
                new Pose2d(t0, theta),
                new Pose2d(t1, theta));
    }

}
