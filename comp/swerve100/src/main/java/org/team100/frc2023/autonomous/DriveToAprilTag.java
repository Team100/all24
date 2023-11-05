package org.team100.frc2023.autonomous;

import java.util.List;
import java.util.function.Supplier;

import org.team100.frc2023.commands.GoalOffset;
import org.team100.lib.controller.PidGains;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToAprilTag extends Command {
    private final Telemetry t = Telemetry.get();

    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_swerve;
    private final SwerveDriveKinematics m_kinematics;
    private final Supplier<GoalOffset> m_goalOffsetSupplier;
    private final Supplier<Double> m_gamePieceOffsetSupplier;
    private final Timer m_timer;

    private final TrajectoryConfig translationConfig;

    private final double m_yOffset;
    private GoalOffset previousOffset;
    private Trajectory m_trajectory;
    private boolean isFinished = false;

    public DriveToAprilTag(
            int tagID,
            double xOffset,
            double yOffset,
            Supplier<GoalOffset> offsetSupplier,
            SwerveDriveSubsystem drivetrain,
            SwerveDriveKinematics kinematics,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Double> gamePieceOffsetSupplier) {
        m_goal = goal(tagID, xOffset, layout);
        m_yOffset = yOffset;
        m_swerve = drivetrain;
        m_kinematics = kinematics;
        m_goalOffsetSupplier = offsetSupplier;
        m_gamePieceOffsetSupplier = gamePieceOffsetSupplier;
        m_timer = new Timer();

        previousOffset = m_goalOffsetSupplier.get();

        translationConfig = new TrajectoryConfig(5, 4.5).setKinematics(kinematics);
        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        isFinished = false;
        m_timer.restart();
        m_trajectory = makeTrajectory(previousOffset, 0);
        m_swerve.setGains(
                new PidGains(2, 0, 0, 0, 0.01, false),
                new PidGains(6.5, 0, 1, 0, 0.01, true));
        m_swerve.setIRange(0.3);
        m_swerve.setTolerance(0.00000001, Math.PI / 180);
    }

    @Override
    public boolean isFinished() {
        return isFinished; // keep trying until the button is released
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public void execute() {
        if (m_trajectory == null) {
            return;
        }
        if (m_goalOffsetSupplier.get() != previousOffset) {
            m_trajectory = makeTrajectory(m_goalOffsetSupplier.get(),
                    m_trajectory.sample(m_timer.get()).velocityMetersPerSecond);
            previousOffset = m_goalOffsetSupplier.get();
            m_timer.restart();
        }
        if (m_trajectory == null) {
            return;
        }

        // TODO: combine xy and theta
        State desiredState = m_trajectory.sample(m_timer.get());
        Rotation2d desiredRot = m_goal.getRotation();

        SwerveState desiredSwerveState = SwerveState.fromState(desiredState, desiredRot);

        t.log("/desired pose/x", desiredSwerveState.x().x());
        t.log("/desired pose/y", desiredSwerveState.y().x());
        t.log("/desired pose/theta", desiredSwerveState.theta().x());

        m_swerve.setDesiredState(desiredSwerveState);
    }

    ///////////////////////////////////////////////////////////////

    static Pose2d goal(int tagID, double xOffset, AprilTagFieldLayoutWithCorrectOrientation layout) {
        Transform2d m_offset = new Transform2d(new Translation2d(-xOffset, 0), new Rotation2d(0));
        Pose2d m_tagPose = layout.getTagPose(tagID).get().toPose2d();
        Pose2d m_goal = m_tagPose.plus(m_offset);
        return m_goal;
    }

    ///////////////////////////////////////////////////////////////

    private Trajectory makeTrajectory(GoalOffset goalOffset, double startVelocity) {
        Pose2d currentPose = m_swerve.getPose();
        Translation2d currentTranslation = currentPose.getTranslation();
        Transform2d goalTransform = new Transform2d();
        if (goalOffset == GoalOffset.left) {
            goalTransform = new Transform2d(new Translation2d(0, -m_yOffset - m_gamePieceOffsetSupplier.get()),
                    new Rotation2d());
        }
        if (goalOffset == GoalOffset.right) {
            goalTransform = new Transform2d(new Translation2d(0, m_yOffset - m_gamePieceOffsetSupplier.get()),
                    new Rotation2d());
        }
        Pose2d transformedGoal = m_goal.plus(goalTransform);
        Translation2d goalTranslation = transformedGoal.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        TrajectoryConfig withStartVelocityConfig = new TrajectoryConfig(5, 2).setKinematics(m_kinematics);
        withStartVelocityConfig.setStartVelocity(startVelocity);

        try {
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(currentTranslation, angleToGoal),
                    List.of(),
                    new Pose2d(goalTranslation, angleToGoal),
                    translationConfig);
        } catch (TrajectoryGenerationException e) {
            isFinished = true;
            return null;
        }
    }
}
