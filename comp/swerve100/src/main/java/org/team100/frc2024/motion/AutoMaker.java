package org.team100.frc2024.motion;

import java.util.List;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
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
    private final double kIntakeOffset = .381;
    private final double kStageOpeningOffset = ;

    public enum FieldPoint {
        NOTE1, NOTE2, NOTE3, NOTE4, NOTE5, NOTE6, NOTE7, NOTE8, WINGSHOT, STAGESHOT, STAGEOPENING
    }

    private Translation2d getTranslation(FieldPoint note) {
        switch (note) {
            case NOTE1:
                return forAlliance(new Translation2d(2.8956, 7.0061), m_alliance);
            case NOTE2:
                return forAlliance(new Translation2d(2.8956, 5.5583), m_alliance);
            case NOTE3:
                return forAlliance(new Translation2d(2.8956, 4.1105), m_alliance);
            case NOTE4:
                return forAlliance(new Translation2d(8.271, 0.7577), m_alliance);
            case NOTE5:
                return forAlliance(new Translation2d(8.271, 2.4341), m_alliance);
            case NOTE6:
                return forAlliance(new Translation2d(8.271, 4.1105), m_alliance);
            case NOTE7:
                return forAlliance(new Translation2d(8.271, 5.7869), m_alliance);
            case NOTE8:
                return forAlliance(new Translation2d(8., 7.4633), m_alliance);
            case WINGSHOT:
                return forAlliance(new Translation2d(5.87248, 6.4), m_alliance);
            case STAGESHOT:
                return forAlliance(new Translation2d(4, 5), m_alliance);
            case STAGEOPENING:
                return forAlliance(new Translation2d(5.87248, 4.1105).plus(new Translation2d(0, kStageOpeningOffset)), m_alliance);
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

    public Pose2d getOffsetPoseWithShooterAngle(FieldPoint note) {
        Translation2d noteTranslation = getTranslation(note);
        Rotation2d endHeading = ShooterUtil.getRobotRotationToSpeaker(noteTranslation, kShooterScale);
        Translation2d offset = new Translation2d(kIntakeOffset * endHeading.getCos(), kIntakeOffset *
                endHeading.getSin());
        return new Pose2d(noteTranslation.plus(offset), endHeading);
    }

    public Pose2d getOffsetPoseWithZeroHeading(FieldPoint note) {
        Translation2d noteTranslation = getTranslation(note);
        Translation2d endTranslation = noteTranslation.minus(new Translation2d(kIntakeOffset, 0));
        return new Pose2d(endTranslation, new Rotation2d(Math.PI));
    }

    public TrajectoryCommand100 adjacentWithShooterAngle(FieldPoint noteA, FieldPoint noteB, double endVelocity) {
        Pose2d noteAOffsetPose = getOffsetPoseWithShooterAngle(noteA);
        Pose2d noteBOffsetPose = getOffsetPoseWithShooterAngle(noteB);
        Translation2d noteAOffsetTranslation = noteAOffsetPose.getTranslation();
        Translation2d noteBOffsetTranslation = noteBOffsetPose.getTranslation();
        Rotation2d angleToGoal = noteBOffsetTranslation.minus(noteAOffsetTranslation).getAngle();
        Pose2d startWaypoint = new Pose2d(noteAOffsetPose.getTranslation(), angleToGoal.times(1.25));
        Pose2d endWaypoint = new Pose2d(noteBOffsetPose.getTranslation(), new Rotation2d());
        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(noteAOffsetPose.getRotation(), noteBOffsetPose.getRotation());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, 0,
                endVelocity, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 adjacentWithShooterAngle(FieldPoint noteA, FieldPoint noteB) {
        return adjacentWithShooterAngle(noteA, noteB, 0);
    }

    public DriveToWithAutoStart startToNote(FieldPoint note) {
        Translation2d noteTranslation = getTranslation(note);
        Rotation2d endHeading = ShooterUtil.getRobotRotationToSpeaker(noteTranslation, kShooterScale);
        Rotation2d endRotation = endHeading.plus(new Rotation2d(Math.PI));
        Translation2d offset = new Translation2d(-.5 * endRotation.getCos(), -.5 * endRotation.getSin());
        // Translation2d offset = new Translation2d();
        Pose2d endWaypoint = new Pose2d(noteTranslation.plus(offset), endRotation);
        return new DriveToWithAutoStart(m_swerve, endWaypoint, endHeading, m_planner, m_controller,
                m_constraints);
    }

    public TrajectoryCommand100 tuningTrajectory1() {
        List<Pose2d> waypointsM = List.of(new Pose2d(2, 2, new Rotation2d()),
                new Pose2d(5, 2, new Rotation2d()));
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d(Math.PI));
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 tuningTrajectory2() {
        List<Pose2d> waypointsM = List.of(new Pose2d(5, 2, new Rotation2d(Math.PI)),
                new Pose2d(2, 2, new Rotation2d(Math.PI)));
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d(Math.PI));
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public DriveToWaypoint100 driveToWingShot() {
        Translation2d wingShotTranslation = getTranslation(FieldPoint.WINGSHOT);
        return new DriveToWaypoint100(
                new Pose2d(wingShotTranslation,
                        ShooterUtil.getRobotRotationToSpeaker(wingShotTranslation, kShooterScale)),
                m_swerve, m_planner, m_controller, m_constraints);
    }

    public TrajectoryCommand100 tuningTrajectory3() {
        List<Pose2d> waypointsM = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S, kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 tuningTrajectory4() {
        List<Pose2d> waypointsM = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d(Math.PI));
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S, kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }


    public TrajectoryCommand100 note1ToNote8() {
        Pose2d startPose = getOffsetPoseWithShooterAngle(FieldPoint.NOTE1);
        Pose2d endPose = getOffsetPoseWithZeroHeading(FieldPoint.NOTE8);
        Rotation2d angleToGoal = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(),
                angleToGoal);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), new Rotation2d());
        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(startPose.getRotation(), endPose.getRotation());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints,
                kMaxVelM_S, kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 note_stageShot(FieldPoint note, boolean toNote) {
        Pose2d startPose = getOffsetPoseWithZeroHeading(note);
        Translation2d startTranslation = startPose.getTranslation();
        Translation2d stageOpeningTranslation = getTranslation(FieldPoint.STAGEOPENING);
        Rotation2d startRotation = stageOpeningTranslation.minus(startTranslation).getAngle();
        if (toNote) {
            startRotation = startRotation.plus(new Rotation2d(Math.PI));
        }
        Pose2d startWaypoint = new Pose2d(startTranslation, startRotation);
        Translation2d stageShotTranslation = getTranslation(FieldPoint.STAGESHOT);
        Rotation2d stageShotHeading = ShooterUtil.getRobotRotationToSpeaker(stageShotTranslation, kShooterScale);
        Rotation2d endRotation = stageShotTranslation.minus(stageOpeningTranslation).getAngle();
        if (toNote) {
            endRotation = endRotation.plus(new Rotation2d(Math.PI));
        }
        Pose2d endWaypoint = new Pose2d(stageShotTranslation, endRotation);
        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), stageShotHeading);
        if (toNote) {
            waypointsM = List.of(endWaypoint, startWaypoint);
            headings = List.of(stageShotHeading, new Rotation2d(Math.PI));
        }
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints,
                kMaxVelM_S, kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public DriveToWaypoint100 driveToNote7() {
        Pose2d goalPose = getOffsetPoseWithZeroHeading(FieldPoint.NOTE7);
        return new DriveToWaypoint100(goalPose, m_swerve, m_planner, m_controller, m_constraints);
    }

    public SequentialCommandGroup eightNoteAuto() {
        return new SequentialCommandGroup(startToNote(FieldPoint.NOTE3), adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
                adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1), note1ToNote8(),
                driveToWingShot(), driveToNote7(),  note_stageShot(FieldPoint.NOTE7, false),
                note_stageShot(FieldPoint.NOTE6, true), note_stageShot(FieldPoint.NOTE6, false),
                note_stageShot(FieldPoint.NOTE5, true), note_stageShot(FieldPoint.NOTE5, false));
    }

    public SequentialCommandGroup fourNoteAuto() {
        return new SequentialCommandGroup(startToNote(FieldPoint.NOTE1),
                adjacentWithShooterAngle(FieldPoint.NOTE1, FieldPoint.NOTE2),
                adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE3));
    }

    public SequentialCommandGroup fiveNoteAuto() {
        return new SequentialCommandGroup(startToNote(FieldPoint.NOTE3),
                adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
                adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1), note1ToNote8(), driveToWingShot());
    }

    public SequentialCommandGroup tuning() {
        return new SequentialCommandGroup(tuningTrajectory3(), new WaitCommand(1), tuningTrajectory4());
    }
}
