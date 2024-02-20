package org.team100.frc2024.motion;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.lib.commands.drivetrain.DriveToState100;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoMaker {
    public SwerveDriveSubsystem m_swerve;
    public TrajectoryPlanner m_planner;
    public DriveMotionController m_controller;
    public SwerveKinodynamics m_limits;
    public AutoMaker(SwerveDriveSubsystem swerve, TrajectoryPlanner planner, DriveMotionController controller, SwerveKinodynamics limits) {
        m_swerve = swerve;
        m_planner = planner;
        m_controller = controller;
        m_limits = limits;
    }
    public Translation2d forAlliance(Translation2d translation, Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return translation;
        }
        return new Translation2d(translation.getX(), 16.542-translation.getY());
    }
    public DriveToWaypoint100 note1(Alliance alliance) {
        Translation2d goalTranslation = forAlliance(new Translation2d(2.5, 6.2), alliance);
        Rotation2d goalRotation = ShooterUtil.getRobotRotationToSpeaker(goalTranslation, .25);
        Pose2d goal = new Pose2d(goalTranslation, goalRotation);
        return new DriveToWaypoint100(goal, m_swerve, m_planner, m_controller, m_limits, () -> ShooterUtil.getRobotRotationToSpeaker(goal.getTranslation(), .25333333333));
    }
}
