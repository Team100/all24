package org.team100.lib.commands;

import java.util.List;

import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DrivePIDController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.planners.TrajectoryPlanner;
import org.team100.lib.swerve.SwerveKinematicLimits;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.trajectory.Trajectory;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FancyTrajectory extends Command {
    private static final double kMaxVelM_S = 4;
    private static final double kMaxAccelM_S_S = 2;
    private static final double kMaxVoltage = 9.0;

    private final Telemetry t = Telemetry.get();

    private final SwerveDriveSubsystemInterface m_robotDrive;
    private final DriveMotionController m_controller;
    private final TrajectoryPlanner m_planner;
    // private final SwerveDriveKinematics m_kinematics;
    // private final SwerveKinematicLimits m_limits;

    public FancyTrajectory(
            SwerveDriveKinematics kinematics,
            SwerveKinematicLimits limits,
            SwerveDriveSubsystemInterface robotDrive) {
        // m_kinematics = kinematics;
        // m_limits = limits;
        m_robotDrive = robotDrive;
        // TODO: try the other follower types.
        // TODO: move this constructor out of here
        m_controller = new DrivePIDController();
        m_planner = new TrajectoryPlanner(kinematics, limits);
        SwerveDriveSubsystem swerveDriveSubsystem = m_robotDrive.get();
        if (swerveDriveSubsystem != null) {
            // it's null in tests.
            addRequirements(swerveDriveSubsystem);
        }
    }

    @Override
    public void initialize() {
        List<Pose2d> waypointsM = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                new Pose2d(80, 80, Rotation2d.fromDegrees(0)));
        // while turning 180
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(0),
                GeometryUtil.fromDegrees(0));
        // these don't actually do anything.
        List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        // note there are static constraints in here.
        // mMotionPlanner = new DriveMotionPlanner();
        double start_vel = 0;
        double end_vel = 0;
        // there's a bug in here; it doesn't use the constraints, nor the voltage.
        Trajectory trajectory = m_planner
                .generateTrajectory(
                        false,
                        waypointsM,
                        headings,
                        constraints,
                        start_vel,
                        end_vel,
                        kMaxVelM_S,
                        kMaxAccelM_S_S,
                        kMaxVoltage);
        // System.out.println(trajectory);
        // System.out.println("TRAJECTORY LENGTH: " + trajectory.length());
        // assertEquals(10, trajectory.length());

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(trajectory));

        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute() {
        final double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_robotDrive.getPose();
        ChassisSpeeds currentSpeed = m_robotDrive.speeds();
        // NOTE(joel): none of the controller implementations actually use the magnitude
        // of the velocity so i'm not sure what unit they expect.
        Twist2d velocity = new Twist2d(
                currentSpeed.vxMetersPerSecond,
                currentSpeed.vyMetersPerSecond,
                currentSpeed.omegaRadiansPerSecond);
        ChassisSpeeds output = m_controller.update(now, currentPose, velocity);
        t.log("/fancy trajectory/chassis speeds", output);
        m_robotDrive.setChassisSpeeds(output);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
