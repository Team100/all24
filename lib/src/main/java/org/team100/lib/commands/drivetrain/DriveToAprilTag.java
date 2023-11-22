package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.PidGains;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** TODO: i think this is the same as DriveToWaypoint3? */
public class DriveToAprilTag extends Command {
    private final Telemetry t = Telemetry.get();
    private final Pose2d m_goal;
    private final SwerveDriveSubsystemInterface m_swerve;
    private final Timer m_timer;
    private final TrajectoryConfig translationConfig;
    private final HolonomicDriveController3 m_controller;
    private Trajectory m_trajectory;

    public DriveToAprilTag(
            int tagID,
            double xOffset,
            double yOffset,
            SwerveDriveSubsystemInterface drivetrain,
            SwerveDriveKinematics kinematics,
            AprilTagFieldLayoutWithCorrectOrientation layout) {
        m_goal = goal(tagID, xOffset, yOffset, layout);
        m_swerve = drivetrain;
        m_timer = new Timer();

        Identity identity = Identity.get();

        DriveControllers controllers = new DriveControllersFactory().get(identity);

        m_controller = new HolonomicDriveController3(controllers);
        m_controller.setTolerance(0.1, 1.0);
        m_controller.setGains(
                new PidGains(2, 0, 0, 0, 0.01, false),
                new PidGains(6.5, 0, 1, 0, 0.01, true));
        m_controller.setIRange(0.3);
        m_controller.setTolerance(0.00000001, Math.PI / 180);

        translationConfig = new TrajectoryConfig(5, 4.5).setKinematics(kinematics);
        if (drivetrain.get() != null)
            addRequirements(drivetrain.get());
    }

    @Override
    public void initialize() {
        m_trajectory = makeTrajectory();
        m_timer.restart();
    }

    @Override
    public void execute() {
        if (m_trajectory == null) {
            return;
        }
        State desiredState = m_trajectory.sample(m_timer.get());
        Pose2d currentPose = m_swerve.getPose();
        // TODO: rotation profile, use new trajectory type
        SwerveState reference = SwerveState.fromState(desiredState, m_goal.getRotation());
        Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);
        m_swerve.driveInFieldCoords(fieldRelativeTarget);
        t.log(Level.DEBUG, "/desired pose/x", reference.x().x());
        t.log(Level.DEBUG, "/desired pose/y", reference.y().x());
        t.log(Level.DEBUG, "/desired pose/theta", reference.theta().x());
    }

    @Override
    public boolean isFinished() {
        if (m_trajectory == null)
            return true;
        return m_timer.get() > m_trajectory.getTotalTimeSeconds() && m_controller.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    ///////////////////////////////////////////////////////////////

    static Pose2d goal(int tagID, double xOffset, double yOffset, AprilTagFieldLayoutWithCorrectOrientation layout) {
        Transform2d m_offset = new Transform2d(new Translation2d(-xOffset, -yOffset), new Rotation2d(0));
        Pose2d m_tagPose = layout.getTagPose(tagID).get().toPose2d();
        return m_tagPose.plus(m_offset);
    }

    ///////////////////////////////////////////////////////////////

    private Trajectory makeTrajectory() {
        Pose2d currentPose = m_swerve.getPose();
        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d goalTranslation = m_goal.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();

        try {
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(currentTranslation, angleToGoal),
                    List.of(),
                    new Pose2d(goalTranslation, angleToGoal),
                    translationConfig);
        } catch (TrajectoryGenerationException e) {
            return null;
        }
    }
}
