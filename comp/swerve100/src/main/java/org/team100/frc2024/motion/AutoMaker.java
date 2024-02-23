package org.team100.frc2024.motion;

import java.util.List;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.commands.drivetrain.TrajectoryCommand;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.sensors.Heading;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryVisualization;
import org.ejml.dense.block.MatrixOps_FDRB;
import org.team100.frc2024.motion.drivetrain.DriveToWithAutoStart;

import edu.wpi.first.math.estimator.MerweScaledSigmaPoints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoMaker {
    public SwerveDriveSubsystem m_swerve;
    public TrajectoryPlanner m_planner;
    public DriveMotionController m_controller;
    public List<TimingConstraint> m_constraints;
    private final double kMaxVelM_S = 4;
    private final double kMaxAccelM_S_S = 5;
    private final double kShooterScale;
    private final Alliance m_alliance;

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
            SwerveKinodynamics limits, double shooterScale, Alliance alliance) {
        m_swerve = swerve;
        m_planner = planner;
        m_controller = controller;
        m_constraints = List.of(
                new CentripetalAccelerationConstraint(limits));
        kShooterScale = shooterScale;
        m_alliance = alliance;
    }

    public Translation2d forAlliance(Translation2d translation, Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return translation;
        }
        return new Translation2d(translation.getX(), 8.221 - translation.getY());
    }

    public Rotation2d forAlliance(Rotation2d rotation, Alliance alliance) {
        return rotation.times(-1);
    }

    public Pose2d forAlliance(Pose2d pose, Alliance alliance) {
        return new Pose2d(forAlliance(pose.getTranslation(), alliance), forAlliance(pose.getRotation(), alliance));
    }

    public Pose2d getOffsetPose(Note note) {
        Translation2d noteTranslation = getTranslation(note, m_alliance);
        Rotation2d endHeading = ShooterUtil.getRobotRotationToSpeaker(noteTranslation, kShooterScale);
        Translation2d offset = new Translation2d(.5 * endHeading.getCos(), .5 *
                endHeading.getSin());
        return new Pose2d(noteTranslation.plus(offset), endHeading);
    }

    public TrajectoryCommand100 adjacentWithShooterAngle(Note noteA, Note noteB) {
        Pose2d noteAOffsetPose = getOffsetPose(noteA);
        Pose2d noteBOffsetPose = getOffsetPose(noteB);
        Translation2d noteAOffsetTranslation = noteAOffsetPose.getTranslation();
        Translation2d noteBOffsetTranslation = noteBOffsetPose.getTranslation();
        Rotation2d angleToGoal = noteBOffsetTranslation.minus(noteAOffsetTranslation).getAngle();
        Pose2d startWaypoint = new Pose2d(noteAOffsetPose.getTranslation(), angleToGoal.times(1.25));
        Pose2d endWaypoint = new Pose2d(noteBOffsetPose.getTranslation(), new Rotation2d());
        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(noteAOffsetPose.getRotation(), noteBOffsetPose.getRotation());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public DriveToWithAutoStart startToNote(Note note) {
        Translation2d noteTranslation = getTranslation(note, m_alliance);
        Rotation2d endHeading = ShooterUtil.getRobotRotationToSpeaker(noteTranslation, kShooterScale);
        Rotation2d endRotation = endHeading.plus(new Rotation2d(Math.PI));
        Translation2d offset = new Translation2d(-.5 * endRotation.getCos(), -.5 * endRotation.getSin());
        // Translation2d offset = new Translation2d();
        Pose2d endWaypoint = new Pose2d(noteTranslation.plus(offset), endRotation);
        return new DriveToWithAutoStart(m_swerve, endWaypoint, endHeading, m_planner, m_controller,
                m_constraints);
    }

    public TrajectoryCommand100 tuningTrajectory() {
        List<Pose2d> waypointsM = List.of(new Pose2d(2, 2, new Rotation2d()),
        new Pose2d(5, 2, new Rotation2d()));
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d(Math.PI));
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S, kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 tuningTrajectory2() {
        List<Pose2d> waypointsM = List.of(new Pose2d(5, 2, new Rotation2d(Math.PI)),
        new Pose2d(2, 2, new Rotation2d(Math.PI)));
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d(Math.PI));
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S, kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 note1_to_note8() {
        Translation2d note1Translation = getTranslation(Note.NOTE1, m_alliance);
        Translation2d note8Translation = getTranslation(Note.NOTE8, m_alliance);
        Pose2d startWaypoint = new Pose2d(note1Translation, new Rotation2d());
        Rotation2d endHeading = ShooterUtil.getRobotRotationToSpeaker(note8Translation, kShooterScale);
        Rotation2d endRotation = endHeading.plus(new Rotation2d(Math.PI));
        Translation2d offset = new Translation2d(-.5 * endRotation.getCos(), -.5 * endRotation.getSin());
        // Translation2d offset = new Translation2d();
        Pose2d endWaypoint = new Pose2d(note8Translation.plus(offset), endRotation);
        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(ShooterUtil.getRobotRotationToSpeaker(note1Translation, kShooterScale),
                endHeading);
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public SequentialCommandGroup fourNoteAuto() {
        return new SequentialCommandGroup(startToNote(Note.NOTE1), adjacentWithShooterAngle(Note.NOTE1, Note.NOTE2),
                adjacentWithShooterAngle(Note.NOTE2, Note.NOTE3));
    }

    public SequentialCommandGroup fiveNoteAuto() {
        return new SequentialCommandGroup(startToNote(Note.NOTE3), adjacentWithShooterAngle(Note.NOTE3, Note.NOTE2),
                adjacentWithShooterAngle(Note.NOTE2, Note.NOTE1), note1_to_note8());
    }

    public SequentialCommandGroup tuning() {
        return new SequentialCommandGroup(tuningTrajectory(), new WaitCommand(1), tuningTrajectory2());
    }
}
