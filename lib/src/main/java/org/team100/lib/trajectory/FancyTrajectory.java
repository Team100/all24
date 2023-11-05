package org.team100.lib.trajectory;

import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.planners.DriveMotionPlanner;
import org.team100.lib.swerve.SwerveKinematicLimits;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;

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

    private final SwerveDriveSubsystem m_robotDrive;
    private final DriveMotionPlanner mMotionPlanner;
    // private final SwerveDriveKinematics m_kinematics;
    // private final SwerveKinematicLimits m_limits;

    public FancyTrajectory(
            SwerveDriveKinematics kinematics,
            SwerveKinematicLimits limits,
            SwerveDriveSubsystem robotDrive) {
        // m_kinematics = kinematics;
        // m_limits = limits;
        m_robotDrive = robotDrive;
        // TODO: try the other follower types.
        // TODO: move this constructor out of here
        mMotionPlanner = new DriveMotionPlanner(kinematics, limits);
        addRequirements(m_robotDrive);
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
        Trajectory trajectory = mMotionPlanner
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
        System.out.println(trajectory);
        System.out.println("TRAJECTORY LENGTH: " + trajectory.length());
        // assertEquals(10, trajectory.length());

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(trajectory));

        mMotionPlanner.reset();
        mMotionPlanner.setTrajectory(iter);
    }

    @Override
    public void execute() {
        final double now = Timer.getFPGATimestamp();

        Pose2d currentPose = new Pose2d(
                m_robotDrive.getPose().getX(),
                m_robotDrive.getPose().getY(),
                m_robotDrive.getPose().getRotation());

        // *****************************************************
        // *****************************************************
        //
        // TODO: measure velocity!
        // SwerveDriveOdometry used to take both positions and velocities; use module
        // velocities to determine chassis speed, and turn that into a twist.
        //
        // *****************************************************
        // *****************************************************
        // *****************************************************

        Twist2d velocity = new Twist2d(); // <<< FIX ME

        ChassisSpeeds output = mMotionPlanner.update(now, currentPose, velocity);
        t.log("/fancy trajectory/chassis speeds", output);
        m_robotDrive.setChassisSpeeds(output);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
