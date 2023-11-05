package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This is a simpler way to drive to a waypoint. It's just like
 * SwerveControllerCommand except that it generates the trajectory at the time
 * the command is scheduled, so it can capture the current robot location at
 * that instant. It runs forever, so it expects to be scheduled via
 * Trigger.whileTrue().
 */
public class DriveToWaypoint2 extends Command {
    private static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(6, 12);

    private final Timer m_timer = new Timer();

    private final Drivetrain m_swerve;
    private final Pose2d goal;

    private final TrajectoryConfig translationConfig;
    private final ProfiledPIDController m_rotationController;
    private final PIDController xController;
    private final PIDController yController;
    private final HolonomicDriveController m_controller;

    private Trajectory m_trajectory;

    public DriveToWaypoint2(Pose2d goal, Drivetrain m_swerve) {
        this.goal = goal;
        this.m_swerve = m_swerve;
        m_rotationController = new ProfiledPIDController(5, 0, 0, rotationConstraints);
        xController = new PIDController(1, 0, 0);
        yController = new PIDController(1, 0, 0);
        m_controller = new HolonomicDriveController(xController, yController, m_rotationController);
        translationConfig = new TrajectoryConfig(5.0, 20.0).setKinematics(m_swerve.m_kinematics);
        addRequirements(m_swerve);
    }

    private Trajectory makeTrajectory() {
        Pose2d currentPose = m_swerve.getPose();
        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d goalTranslation = goal.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();

        try {
        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(currentTranslation, angleToGoal),
                List.of(),
                new Pose2d(goalTranslation, angleToGoal),
                translationConfig);
        } catch (TrajectoryParameterizer.TrajectoryGenerationException e) {
            return null;
        }
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_trajectory = makeTrajectory();
    }

    @Override
    public boolean isFinished() {
        return false; // keep trying until the button is released
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public void execute() {
        if (m_trajectory == null) return;
        double curTime = m_timer.get();
        var desiredState = m_trajectory.sample(curTime);

        var targetChassisSpeeds = m_controller.calculate(m_swerve.getPose(), desiredState, goal.getRotation());
        var targetModuleStates = m_swerve.m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_swerve.setModuleStates(targetModuleStates);
    }

}
