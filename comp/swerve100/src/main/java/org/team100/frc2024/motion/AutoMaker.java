package org.team100.frc2024.motion;

import java.util.List;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoMaker {
    public SwerveDriveSubsystem m_swerve;
    public TrajectoryPlanner m_planner;
    public DriveMotionController m_controller;
    public List<TimingConstraint> m_constraints;
    private final double kMaxVelM_S = 4;
    private final double kMaxAccelM_S_S = 4;
    private final double kShooterScale;

    public enum Note {
        NOTE1, NOTE2, NOTE3, NOTE4, NOTE5, NOTE6, NOTE7, NOTE8
    }

    private getTranslation(Note note) {
        switch (note) {
            case NOTE1:
                return new Translation2d();
            case NOTE2:
                return new Translation2d();
            case NOTE3:
                return new Translation2d();
            case NOTE4:
                return new Translation2d();
            case NOTE5:
                return new Translation2d();
            case NOTE6:
                return new Translation2d();
            case NOTE7:
                return new Translation2d();
            case NOTE8:
                return new Translation2d();
        }
    }

    public AutoMaker(SwerveDriveSubsystem swerve, TrajectoryPlanner planner, DriveMotionController controller,
            SwerveKinodynamics limits, double shooterScale) {
        m_swerve = swerve;
        m_planner = planner;
        m_controller = controller;
        m_constraints = List.of(
                new CentripetalAccelerationConstraint(limits));
        kShooterScale = shooterScale;
    }

    public Translation2d forAlliance(Translation2d translation, Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return translation;
        }
        return new Translation2d(translation.getX(), 16.542 - translation.getY());
    }

    public Rotation2d forAlliance(Rotation2d rotation, Alliance alliance) {
        return rotation.times(-1);
    }

    public Pose2d forAlliance(Pose2d pose, Alliance alliance) {
        return new Pose2d(forAlliance(pose.getTranslation(), alliance), forAlliance(pose.getRotation(), alliance));
    }

    public Pose2d waypointWithShooterAngle(Translation2d translation, Alliance alliance, boolean reversed) {
        if (reversed) {
            return new Pose2d(forAlliance(translation, alliance),
                    ShooterUtil.getRobotRotationToSpeaker(forAlliance(translation, alliance), kShooterScale)
                            .plus(new Rotation2d(Math.PI)));
        }
        return new Pose2d(forAlliance(translation, alliance),
                ShooterUtil.getRobotRotationToSpeaker(forAlliance(translation, alliance), kShooterScale));

    }

    public TrajectoryCommand100 adjacentWithShooterAngle(Note noteA, Note noteB, Alliance alliance) {
    }
}
