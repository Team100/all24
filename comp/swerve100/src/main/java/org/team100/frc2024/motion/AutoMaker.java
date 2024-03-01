package org.team100.frc2024.motion;

import java.util.List;
import java.util.Optional;

import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.drivetrain.DriveToWithAutoStart;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoMaker {
    public SwerveDriveSubsystem m_swerve;
    public TrajectoryPlanner m_planner;
    public DriveMotionController m_controller;
    public SensorInterface m_sensors;
    public List<TimingConstraint> m_constraints;

  public Intake m_intake;
    public Shooter m_shooter;
    public FeederSubsystem m_feeder;
    private final double kMaxVelM_S = 2;
    private final double kMaxAccelM_S_S = 2;

  private final NotePosition24ArrayListener m_notePosition24ArrayListener;
    private final double kAutoNoteMaxVelM_S = 4;
    private final double kAutoNoteMaxAccelM_S_S = 5;

  private final double kShooterScale;
    private final Alliance m_alliance;
    private final double kIntakeOffset = 0;

    public enum FieldPoint {
        NOTE1, NOTE2, NOTE3, NOTE4, NOTE5, NOTE6, NOTE7, NOTE8, CLOSEWINGSHOT, FARWINGSHOT, STAGESHOT,
        CENTRALSTAGEOPENING,
        FARSTAGEADJACENT, FARSTAGEOPENING, DROPSHOT, CLOSESTAGEADJACENT
    }

    private Translation2d getTranslation(FieldPoint point) {
        if (Experiments.instance.enabled(Experiment.AutoNoteDetection)) {
            Translation2d pose;
            Optional<Translation2d> translation2d;
            switch (point) {
                case NOTE1:
                    pose = forAlliance(new Translation2d(2.8956, 7.0061), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case NOTE2:
                    pose = forAlliance(new Translation2d(2.8956, 5.5583), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case NOTE3:
                    pose = forAlliance(new Translation2d(2.8956, 4.1105), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case NOTE4:
                    pose = forAlliance(new Translation2d(8.271, 0.7577), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case NOTE5:
                    pose = forAlliance(new Translation2d(8.271, 2.4341), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case NOTE6:
                    pose = forAlliance(new Translation2d(8.271, 4.1105), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case NOTE7:
                    pose = forAlliance(new Translation2d(8.271, 5.7869), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case NOTE8:
                    pose = forAlliance(new Translation2d(8.271, 7.4633), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case CLOSEWINGSHOT:
                    pose =forAlliance(new Translation2d(5.87248, 6.4), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case FARWINGSHOT:
                    pose = forAlliance(new Translation2d(4, 1.5), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case STAGESHOT:
                    pose = forAlliance(new Translation2d(4.25, 5), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case CENTRALSTAGEOPENING:
                    pose = forAlliance(new Translation2d(5.87248, 4.1105), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case FARSTAGEOPENING:
                    pose = forAlliance(new Translation2d(4.3, 3.3), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                case DROPSHOT:
                    pose = forAlliance(new Translation2d(.5, 1.8), m_alliance);
                    translation2d = m_notePosition24ArrayListener.getTranslation2dAuto(pose);
                    if (translation2d.isPresent()) {
                        return translation2d.get();
                    }
                    return pose;
                default:
                    return new Translation2d();
            }
        }
        switch (point) {
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
                return forAlliance(new Translation2d(8.271, 7.4633), m_alliance);
            case CLOSEWINGSHOT:
                return forAlliance(new Translation2d(3, 6.4), m_alliance);
            case FARWINGSHOT:
                return forAlliance(new Translation2d(4, 1.5), m_alliance);
            case STAGESHOT:
                return forAlliance(new Translation2d(4.25, 5), m_alliance);
            case CENTRALSTAGEOPENING:
                return forAlliance(new Translation2d(5.87248, 4.1105), m_alliance);
            case FARSTAGEOPENING:
                return forAlliance(new Translation2d(4.3, 3.3), m_alliance);
            case FARSTAGEADJACENT:
                return forAlliance(new Translation2d(5.87248, 1.9), m_alliance);
            case CLOSESTAGEADJACENT:
                return forAlliance(new Translation2d(5.87248, 6.45), m_alliance);
            case DROPSHOT:
                return forAlliance(new Translation2d(.5, 1.8), m_alliance);
            default:
                return new Translation2d();
        }
    }

    private Pose2d getPose(FieldPoint point) {
        Translation2d translation = getTranslation(point);
        Rotation2d heading = new Rotation2d(Math.PI);
        switch (point) {
            case NOTE1, NOTE2:
                heading = ShooterUtil.getRobotRotationToSpeaker(translation, kShooterScale);
                Translation2d offset = new Translation2d(kIntakeOffset * heading.getCos(), kIntakeOffset *
                        heading.getSin());
                return new Pose2d(translation.plus(offset), heading);
            case NOTE3:
                heading = ShooterUtil.getRobotRotationToSpeaker(translation, kShooterScale);
                Translation2d note3Offset = new Translation2d(.2 * heading.getCos(), .2 *
                        heading.getSin());
                return new Pose2d(translation.plus(note3Offset), heading);
            case CLOSEWINGSHOT, FARWINGSHOT, STAGESHOT, DROPSHOT:
                heading = ShooterUtil.getRobotRotationToSpeaker(translation, kShooterScale);
                return new Pose2d(translation, heading);
            case CENTRALSTAGEOPENING, FARSTAGEOPENING, FARSTAGEADJACENT:
                return new Pose2d(translation, heading);
            default:
                return new Pose2d(translation.plus(new Translation2d(-kIntakeOffset, 0)), heading);
        }
    }

    public AutoMaker(SwerveDriveSubsystem swerve, TrajectoryPlanner planner, DriveMotionController controller,
            SwerveKinodynamics limits, double shooterScale, Alliance alliance, FeederSubsystem feeder, Shooter shooter,
            Intake intake, SensorInterface sensor, NotePosition24ArrayListener notePosition24ArrayListener) {
        m_notePosition24ArrayListener = notePosition24ArrayListener;
        m_swerve = swerve;
        m_planner = planner;
        m_controller = controller;
        m_constraints = List.of(
                new CentripetalAccelerationConstraint(limits));
        kShooterScale = shooterScale;
        m_feeder = feeder;
        m_shooter = shooter;
        m_intake = intake;
        m_sensors = sensor;
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

    public TrajectoryCommand100 adjacentWithShooterAngle(FieldPoint noteA, FieldPoint noteB) {
        Pose2d startPose = getPose(noteA);
        Pose2d endPose = getPose(noteB);
        Translation2d translationToGoal = endPose.getTranslation().minus(startPose.getTranslation());
        Translation2d betweenOffset = translationToGoal.times(-.25).plus(new Translation2d(-.875, 0));
        Rotation2d angleToGoal = translationToGoal.getAngle();
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), angleToGoal.times(1.375));
        Pose2d betweenWaypoint = new Pose2d(endPose.getTranslation().plus(betweenOffset), angleToGoal);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), endPose.getRotation().plus(new Rotation2d(Math.PI)));
        List<Pose2d> waypointsM = List.of(startWaypoint, betweenWaypoint, endWaypoint);
        Rotation2d betweenHeading = new Rotation2d(Math.PI);
        List<Rotation2d> headings = List.of(startPose.getRotation(), betweenHeading, endPose.getRotation());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public DriveToWithAutoStart startToNote(FieldPoint note) {
        Pose2d endPose = getPose(note);
        Rotation2d endRotation = new Rotation2d(Math.PI / 2);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), endRotation);
        return new DriveToWithAutoStart(m_swerve, endWaypoint, endPose.getRotation(), m_planner, m_controller,
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

    public TrajectoryCommand100 tuningTrajectory3() {
        List<Pose2d> waypointsM = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 tuningTrajectory4() {
        List<Pose2d> waypointsM = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d(Math.PI));
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 stageManeuver(FieldPoint start, FieldPoint between, FieldPoint end) {
        Pose2d startPose = getPose(start);
        Pose2d betweenPose = getPose(between);
        Pose2d endPose = getPose(end);

        Rotation2d startRotationToOpening = betweenPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        Rotation2d betweenRotation = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        Rotation2d endRotationFromOpening = endPose.getTranslation().minus(betweenPose.getTranslation()).getAngle();
        Rotation2d startRotation = startRotationToOpening.minus(betweenRotation).times(1.5).plus(betweenRotation);
        Rotation2d endRotation = endRotationFromOpening.minus(betweenRotation).times(1.5).plus(betweenRotation);
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), startRotation);
        Pose2d betweenWaypoint = new Pose2d(betweenPose.getTranslation(), betweenRotation);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), endRotation);
        List<Pose2d> waypointsM = List.of(startWaypoint, betweenWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(startPose.getRotation(), betweenPose.getRotation(), endPose.getRotation());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 throughCentralStageOpening(FieldPoint start, FieldPoint end) {
        return stageManeuver(start, FieldPoint.CENTRALSTAGEOPENING, end);
    }

    public TrajectoryCommand100 throughFarStageOpening(FieldPoint start, FieldPoint end) {
        return stageManeuver(start, FieldPoint.FARSTAGEOPENING, end);
    }

    public TrajectoryCommand100 aroundStageClose(FieldPoint start, FieldPoint end) {
        return stageManeuver(start, FieldPoint.CLOSESTAGEADJACENT, end);
    }

    public TrajectoryCommand100 aroundStageFar(FieldPoint start, FieldPoint end) {
        return stageManeuver(start, FieldPoint.FARSTAGEADJACENT, end);
    }

    public DriveToWaypoint100 driveToStraight(FieldPoint point) {
        return new DriveToWaypoint100(getPose(point), m_swerve, m_planner, m_controller, m_constraints, 1);
    }

    public SequentialCommandGroup eightNoteAuto() {
        return new SequentialCommandGroup(startToNote(FieldPoint.NOTE3),
                adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
                adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1), driveToStraight(FieldPoint.NOTE8),
                driveToStraight(FieldPoint.CLOSEWINGSHOT), aroundStageClose(FieldPoint.CLOSEWINGSHOT, FieldPoint.NOTE7),
                throughCentralStageOpening(FieldPoint.NOTE7, FieldPoint.STAGESHOT),
                throughCentralStageOpening(FieldPoint.STAGESHOT, FieldPoint.NOTE6),
                throughCentralStageOpening(FieldPoint.NOTE6, FieldPoint.STAGESHOT),
                throughCentralStageOpening(FieldPoint.NOTE5, FieldPoint.STAGESHOT),
                throughCentralStageOpening(FieldPoint.STAGESHOT, FieldPoint.NOTE5));
    }

    public SequentialCommandGroup complementAuto() {
        return new SequentialCommandGroup(driveToStraight(FieldPoint.NOTE4), driveToStraight(FieldPoint.FARWINGSHOT),
                aroundStageClose(FieldPoint.FARWINGSHOT, FieldPoint.NOTE5),
                throughCentralStageOpening(FieldPoint.NOTE5, FieldPoint.STAGESHOT),
                throughFarStageOpening(FieldPoint.STAGESHOT, FieldPoint.DROPSHOT));
    }

    public SequentialCommandGroup fourNoteAuto() {
        // return new SequentialCommandGroup(
        //         new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, true),
        //         new ParallelCommandGroup(driveToStraight(FieldPoint.NOTE3),
        //                 new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
        //         new ParallelCommandGroup(adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
        //                 new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
        //         new ParallelCommandGroup(adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1),
        //                 new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)));

        return new SequentialCommandGroup(

                new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, true),

                new ParallelRaceGroup( driveToStraight(FieldPoint.NOTE3),
                        new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),

                new ParallelRaceGroup(adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
                        new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
                new ParallelRaceGroup(adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1),
                        new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)));

        // return new SequentialCommandGroup(
        //         // new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, true),
        //         driveToStraight(FieldPoint.NOTE3),
        //         adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
        //         adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1));
    }

    // public Command getTimeoutCommand(Command trajectoryCommand,  ){
        
    // }

    public SequentialCommandGroup fiveNoteAuto() {
        return new SequentialCommandGroup(
                new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, true),
                new ParallelRaceGroup( driveToStraight(FieldPoint.NOTE3),
                        new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
                new ParallelRaceGroup(adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
                        new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
                new ParallelRaceGroup(adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1),
                        new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
                driveToStraight(FieldPoint.NOTE8),
                driveToStraight(FieldPoint.CLOSEWINGSHOT));
    }

    public SequentialCommandGroup tuning() {
        return new SequentialCommandGroup(tuningTrajectory1(), new WaitCommand(1), tuningTrajectory2());
    }
}