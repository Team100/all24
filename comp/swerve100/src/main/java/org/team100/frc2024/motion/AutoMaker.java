package org.team100.frc2024.motion;

import java.util.List;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.sensors.Heading;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
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

    private Translation2d getTranslation(Note note, Alliance alliance) {
        switch (note) {
            case NOTE1:
                return forAlliance(new Translation2d(2.8956, 7.0061), alliance);
            case NOTE2:
                return forAlliance(new Translation2d(2.8956, 5.5583), alliance);
            case NOTE3:
                return forAlliance(new Translation2d(2.8956, 4.1105), alliance);
            case NOTE4:
                return forAlliance(new Translation2d(8.271, 0.7577), alliance);
            case NOTE5:
                return forAlliance(new Translation2d(8.271, 2.4341), alliance);
            case NOTE6:
                return forAlliance(new Translation2d(8.271, 4.1105), alliance);
            case NOTE7:
                return forAlliance(new Translation2d(8.271, 5.7869), alliance);
            case NOTE8:
                return forAlliance(new Translation2d(8., 7.4633), alliance);
            default:
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

    public TrajectoryCommand100 adjacentWithShooterAngle(Note noteA, Note noteB, Alliance alliance) {
        Translation2d noteATranslation = getTranslation(noteA, alliance);
        Translation2d noteBTranslation = getTranslation(noteB, alliance);
        Rotation2d rotationToGoal = noteBTranslation.minus(noteATranslation).getAngle();
        System.out.println(rotationToGoal);
        Rotation2d startRotation = rotationToGoal.times(1.5);
        Pose2d startWaypoint = new Pose2d(noteATranslation, startRotation);
        Rotation2d endHeading = ShooterUtil.getRobotRotationToSpeaker(noteBTranslation, kShooterScale);
        Rotation2d endRotation = endHeading.plus(new Rotation2d(Math.PI));
        Translation2d offset = new Translation2d(-1 * endRotation.getCos(), -1 * endRotation.getSin());
        Pose2d endWaypoint = new Pose2d(noteBTranslation.plus(offset), endRotation);
        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(ShooterUtil.getRobotRotationToSpeaker(noteBTranslation, kShooterScale), endHeading);
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S, kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }
}
