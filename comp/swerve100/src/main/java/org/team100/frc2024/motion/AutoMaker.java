package org.team100.frc2024.motion;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;

import org.team100.frc2024.CompSensors;
import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.amp.AmpSubsystem;
import org.team100.frc2024.motion.drivetrain.DriveToWithAutoStart;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.RotateToShooter;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.commands.drivetrain.DriveWithProfile2;
import org.team100.lib.commands.drivetrain.DriveWithProfileNote;
import org.team100.lib.commands.drivetrain.Rotate;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DriveMotionControllerFactory;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.sensors.HeadingInterface;
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
    public AmpSubsystem m_amp;
    public SwerveKinodynamics m_limits;


  public Intake m_intake;
    public Shooter m_shooter;
    public FeederSubsystem m_feeder;
    private final double kMaxVelM_S = 2;
    private final double kMaxAccelM_S_S = 2;

  private final NotePosition24ArrayListener m_notePosition24ArrayListener;
    private final double kAutoNoteMaxVelM_S = 2;
    private final double kAutoNoteMaxAccelM_S_S = 2;

    HeadingInterface m_heading;

  private final double kShooterScale;
    private final Alliance m_alliance;
    private final double kIntakeOffset = 0;

    public enum FieldPoint {
        NOTE1, NOTE2, NOTE3, NOTE4, NOTE5, NOTE6, NOTE7, NOTE8, CLOSEWINGSHOT, FARWINGSHOT, STAGESHOT,
        CENTRALSTAGEOPENING,
        FARSTAGEADJACENT, FARSTAGEOPENING, DROPSHOT, CLOSESTAGEADJACENT, STARTSUBWOOFER, DRIVETONOTEHANDOFF, CITRUSMID, CITRUSEND, CITRUSBEGIN
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
                    return pose;
                case FARWINGSHOT:
                    pose = forAlliance(new Translation2d(4, 1.5), m_alliance);
                    return pose;
                case STAGESHOT:
                    pose = forAlliance(new Translation2d(4.25, 5), m_alliance);
                    return pose;
                case CENTRALSTAGEOPENING:
                    pose = forAlliance(new Translation2d(5.87248, 4.1105), m_alliance);
                    return pose;
                case FARSTAGEOPENING:
                    pose = forAlliance(new Translation2d(4.3, 3.3), m_alliance);
                    return pose;
                case DROPSHOT:
                    pose = forAlliance(new Translation2d(.5, 1.8), m_alliance);
                    return pose;
                case DRIVETONOTEHANDOFF:
                    pose = forAlliance(new Translation2d(6.2,7.8), m_alliance);
                    return pose;
                default:
                    return new Translation2d();
            }
        }
        switch (point) {
            case NOTE1:
                return forAlliance(new Translation2d(2.8956, 7.0061), m_alliance); //7.0061
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
                return forAlliance(new Translation2d(8.271, 8.3633), m_alliance); //WAS 7.4
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
            case STARTSUBWOOFER:
                return forAlliance(new Translation2d(1.38, 5.566847), m_alliance);
            case DRIVETONOTEHANDOFF:
                    return forAlliance(new Translation2d(4.9, 7.5  ), m_alliance);
            case CITRUSMID:
                    return forAlliance(new Translation2d(3.269011, 1.306543), m_alliance);
            case CITRUSEND:
                    return forAlliance(new Translation2d(6.964483, 1.476407), m_alliance);
            case CITRUSBEGIN:
                    return forAlliance( new Translation2d(0.865358, 4.215958), m_alliance);
            default:
                return new Translation2d();
        }
    }

    private Pose2d getPose(Alliance alliance, FieldPoint point) {
        Translation2d translation = getTranslation(point);
        Rotation2d heading = new Rotation2d(Math.PI);
        switch (point) {
            case NOTE1, NOTE2:
                heading = ShooterUtil.getRobotRotationToSpeaker(alliance, translation, kShooterScale);
                // heading = new Rotation2d(Math.PI);
                Translation2d offset = new Translation2d(kIntakeOffset * heading.getCos(), kIntakeOffset *
                        heading.getSin());
                return new Pose2d(translation.plus(offset), heading);
            case NOTE3:
                heading = ShooterUtil.getRobotRotationToSpeaker(alliance, translation, kShooterScale);
                Translation2d note3Offset = new Translation2d(.2 * heading.getCos(), .2 *
                        heading.getSin());
                return new Pose2d(translation.plus(note3Offset), heading);
            case CLOSEWINGSHOT, FARWINGSHOT, STAGESHOT, DROPSHOT:
                heading = ShooterUtil.getRobotRotationToSpeaker(alliance, translation, kShooterScale);
                return new Pose2d(translation, heading);
            case CENTRALSTAGEOPENING, FARSTAGEOPENING, FARSTAGEADJACENT, DRIVETONOTEHANDOFF:
                return new Pose2d(translation, heading);
            default:
                return new Pose2d(translation.plus(new Translation2d(-kIntakeOffset, 0)), heading);
        }
    }

    public AutoMaker(SwerveDriveSubsystem swerve, TrajectoryPlanner planner, DriveMotionController controller,
            SwerveKinodynamics limits, double shooterScale, Alliance alliance, FeederSubsystem feeder, Shooter shooter,
            Intake intake, SensorInterface sensor, NotePosition24ArrayListener notePosition24ArrayListener, AmpSubsystem amp, HeadingInterface heading, List<TimingConstraint> constraints) {
        m_notePosition24ArrayListener = notePosition24ArrayListener;
        m_swerve = swerve;
        m_planner = planner;
        m_controller = controller;
        m_constraints = constraints;
        kShooterScale = shooterScale;
        m_feeder = feeder;
        m_shooter = shooter;
        m_intake = intake;
        m_sensors = sensor;
        m_alliance = alliance;
        m_amp = amp;
        m_heading = heading;
        m_limits = limits;
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

    public TrajectoryCommand100 adjacentWithShooterAngle(Alliance alliance, FieldPoint noteA, FieldPoint noteB) {
        Pose2d startPose = getPose(alliance, noteA);
        Pose2d endPose = getPose(alliance, noteB);
        Translation2d translationToGoal = endPose.getTranslation().minus(startPose.getTranslation());
        Translation2d betweenOffset = translationToGoal.times(-.5).plus(new Translation2d(-.875, 0));
        Rotation2d angleToGoal = translationToGoal.getAngle();
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), angleToGoal.times(1.75));
        Pose2d betweenWaypoint = new Pose2d(endPose.getTranslation().plus(betweenOffset), angleToGoal);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), endPose.getRotation().plus(new Rotation2d(Math.PI)));
        List<Pose2d> waypointsM = List.of(startWaypoint, betweenWaypoint, endWaypoint);
        Rotation2d betweenHeading = new Rotation2d(Math.PI);
        List<Rotation2d> headings = List.of(startPose.getRotation(), betweenHeading, endPose.getRotation());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

public TrajectoryCommand100 test(Alliance alliance, FieldPoint noteA, Translation2d waypoint, Translation2d waypoint2, FieldPoint noteB) {
        Pose2d startPose = getPose(alliance, noteA);
        Pose2d endPose = getPose(alliance, noteB);
        Translation2d translationToGoal = endPose.getTranslation().minus(startPose.getTranslation());

        Translation2d translationGoalToWaypoint = endPose.getTranslation().minus(waypoint);
        

        Rotation2d angleFromWaypoint = translationGoalToWaypoint.getAngle();

        Rotation2d angleToGoal = translationToGoal.getAngle();
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), Rotation2d.fromDegrees(170));


        Pose2d betweenWaypoint = new Pose2d(waypoint, waypoint2.minus(waypoint).getAngle());

        Pose2d waypoint3 = new Pose2d(new Translation2d(1.57, 4.67), Rotation2d.fromDegrees(90));

        Pose2d midWaypoint = new Pose2d(waypoint2, endPose.getTranslation().minus(waypoint2).getAngle());

        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), endPose.getRotation().plus(new Rotation2d(Math.PI)));

        List<Pose2d> waypointsM = List.of(startWaypoint, betweenWaypoint, midWaypoint, endWaypoint);
        Rotation2d betweenHeading = new Rotation2d(ShooterUtil.getRobotRotationToSpeaker(alliance, waypoint, 0).getRadians());
        System.out.println("ENNNNNNNNNNNNNNDDDDDDDDDDDDDD" + endPose.getRotation());
        List<Rotation2d> headings = List.of(startPose.getRotation(), betweenHeading, betweenHeading, endPose.getRotation());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kMaxVelM_S,
                kMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    

    public DriveToWithAutoStart startToNote(Alliance alliance, FieldPoint note) {
        Pose2d endPose = getPose(alliance, note);
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

    public TrajectoryCommand100 tuningTrajectory6() {
        List<Pose2d> waypointsM = List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                new Pose2d(1, 1, Rotation2d.fromDegrees(45)));

        List<Rotation2d> headings = List.of(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(150));

        //  List<Pose2d> waypointsM = List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        //         new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        // List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d(0));


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

    public TrajectoryCommand100 driveToStageBase(Alliance alliance, FieldPoint start, FieldPoint end) {
        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);
        Rotation2d angleToGoal = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        
        Pose2d waypoint = new Pose2d( forAlliance(new Translation2d(1.92, 4.84), m_alliance), angleToGoal);
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), waypoint.getTranslation().minus(startPose.getTranslation()).getAngle());
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);


        List<Pose2d> waypointsM = List.of(startWaypoint, waypoint, endWaypoint);
        List<Rotation2d> headings = List.of(startPose.getRotation(), endPose.getRotation(), endPose.getRotation());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kAutoNoteMaxVelM_S, kAutoNoteMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 driveStraight(Alliance alliance, FieldPoint start, FieldPoint end) {
        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);
        Rotation2d angleToGoal = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), angleToGoal);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);
        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(startPose.getRotation(), endPose.getRotation());
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kAutoNoteMaxVelM_S, kAutoNoteMaxAccelM_S_S);
        return new TrajectoryCommand100(m_swerve, trajectory, m_controller);
    }

    public TrajectoryCommand100 driveStraight(Pose2d start, Pose2d end) {

        Pose2d startPose = start;
        Pose2d endPose = end;

        Rotation2d angleToGoal = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), angleToGoal);

        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);


        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(startPose.getRotation(), endPose.getRotation());

//       Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, kAutoNoteMaxVelM_S, kAutoNoteMaxAccelM_S_S);
//         return new TrajectoryCommand100(m_swerve, trajec tory, new DrivePIDFController(false, 2, 2));

      Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints,
                kAutoNoteMaxVelM_S, kAutoNoteMaxAccelM_S_S);
        return new TrajectoryCommand100(
                m_swerve,
                trajectory,
                DriveMotionControllerFactory.goodPIDF());
    }

    public TrajectoryCommand100 driveStraightWithWaypoints(Alliance alliance, FieldPoint start, Translation2d waypoint, FieldPoint end, Rotation2d endingSplineDirection) {
        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);
        Rotation2d angleToGoal = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), endPose.getTranslation().minus(startPose.getTranslation()).getAngle());
        Pose2d midPoint = new Pose2d(waypoint, endingSplineDirection);

        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(),  endPose.getTranslation().minus(startPose.getTranslation()).getAngle());


        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(startPose.getRotation(), new Rotation2d(Math.PI));
        Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints, 2, 2);
        return new TrajectoryCommand100(m_swerve, trajectory, DriveMotionControllerFactory.goodPIDF());

//         List<Pose2d> waypointsM = List.of(startWaypoint, midPoint, endWaypoint);
//         List<Rotation2d> headings = List.of(startPose.getRotation(), endPose.getRotation(), endPose.getRotation());
//         Trajectory100 trajectory = m_planner.generateTrajectory(false, waypointsM, headings, m_constraints,
//                 kAutoNoteMaxVelM_S, kAutoNoteMaxAccelM_S_S);
//         // joel 20240311 changed ptheta from 2 to 1.3
//         return new TrajectoryCommand100(
//                 m_swerve,
//                 trajectory,
//                 DriveMotionControllerFactory.goodPIDF());
    }

    public TrajectoryCommand100 stageManeuver(Alliance alliance, FieldPoint start, FieldPoint between, FieldPoint end) {
        Pose2d startPose = getPose(alliance, start);
        Pose2d betweenPose = getPose(alliance, between);
        Pose2d endPose = getPose(alliance, end);

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

    public TrajectoryCommand100 throughCentralStageOpening(Alliance alliance, FieldPoint start, FieldPoint end) {
        return stageManeuver(alliance, start, FieldPoint.CENTRALSTAGEOPENING, end);
    }

    public TrajectoryCommand100 throughFarStageOpening(Alliance alliance, FieldPoint start, FieldPoint end) {
        return stageManeuver(alliance, start, FieldPoint.FARSTAGEOPENING, end);
    }

    public TrajectoryCommand100 aroundStageClose(Alliance alliance, FieldPoint start, FieldPoint end) {
        return stageManeuver(alliance, start, FieldPoint.CLOSESTAGEADJACENT, end);
    }

    public TrajectoryCommand100 aroundStageFar(Alliance alliance, FieldPoint start, FieldPoint end) {
        return stageManeuver(alliance, start, FieldPoint.FARSTAGEADJACENT, end);
    }

    public DriveToWaypoint100 driveToStraight(Alliance alliance, FieldPoint point) {
        return new DriveToWaypoint100(getPose(alliance, point), m_swerve, m_planner, m_controller, m_constraints, 1);
    }

    public SequentialCommandGroup eightNoteAuto(Alliance alliance) {
        return new SequentialCommandGroup(startToNote(alliance, FieldPoint.NOTE3),
                adjacentWithShooterAngle(alliance, FieldPoint.NOTE3, FieldPoint.NOTE2),
                adjacentWithShooterAngle(alliance, FieldPoint.NOTE2, FieldPoint.NOTE1), driveToStraight(alliance, FieldPoint.NOTE8),
                driveToStraight(alliance, FieldPoint.CLOSEWINGSHOT), aroundStageClose(alliance, FieldPoint.CLOSEWINGSHOT, FieldPoint.NOTE7),
                throughCentralStageOpening(alliance, FieldPoint.NOTE7, FieldPoint.STAGESHOT),
                throughCentralStageOpening(alliance, FieldPoint.STAGESHOT, FieldPoint.NOTE6),
                throughCentralStageOpening(alliance, FieldPoint.NOTE6, FieldPoint.STAGESHOT),
                throughCentralStageOpening(alliance, FieldPoint.NOTE5, FieldPoint.STAGESHOT),
                throughCentralStageOpening(alliance, FieldPoint.STAGESHOT, FieldPoint.NOTE5));
    }

    public SequentialCommandGroup complementAuto(Alliance alliance) {
        return new SequentialCommandGroup(driveToStraight(alliance, FieldPoint.NOTE4), driveToStraight(alliance, FieldPoint.FARWINGSHOT),
                aroundStageFar(alliance, FieldPoint.FARWINGSHOT, FieldPoint.NOTE5),
                throughCentralStageOpening(alliance, FieldPoint.NOTE5, FieldPoint.STAGESHOT),
                throughFarStageOpening(alliance, FieldPoint.STAGESHOT, FieldPoint.DROPSHOT));
    }

    public SequentialCommandGroup fourNoteAuto(NotePosition24ArrayListener noteDetecor,SwerveKinodynamics limits,SensorInterface sensor) {
        // return new SequentialCommandGroup(
        //         new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, true),
        //         new ParallelCommandGroup(driveToStraight(FieldPoint.NOTE3),
        //                 new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
        //         new ParallelCommandGroup(adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
        //                 new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
        //         new ParallelCommandGroup(adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1),
        //                 new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)));

        // return new SequentialCommandGroup(
                //         new StowAmpCommand(m_amp),
                //         new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, true)
    
                //         // new ParallelRaceGroup( driveStraight(FieldPoint.STARTSUBWOOFER, FieldPoint.NOTE3),
                //         //         new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),

                //         // new ParallelRaceGroup(adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
                //         //         new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
                //         // new ParallelRaceGroup(adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1),
                //         //         new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)));
        // );

        // return new SequentialCommandGroup(
        //         // new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, true),
        //         driveStraight(FieldPoint.STARTSUBWOOFER, FieldPoint.NOTE3),
        //         adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
        //         adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1));

        // return new SequentialCommandGroup(
        //     new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, -1),
        //     new ParallelRaceGroup(driveToStageBase(FieldPoint.STARTSUBWOOFER, FieldPoint.NOTE3), new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, -1)), 
            
        //     new ParallelRaceGroup(test(FieldPoint.NOTE3, forAlliance(new Translation2d(1.99, 5.5583), m_alliance), forAlliance(new Translation2d(2.3, 5.5583), m_alliance), FieldPoint.NOTE2),  new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, -1)),
            
        //     new ParallelRaceGroup(test(FieldPoint.NOTE2, forAlliance(new Translation2d(1.95, 6.47), m_alliance), forAlliance(new Translation2d(2.307, 6.67), m_alliance), FieldPoint.NOTE1),  new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, -1)),
        //     // new ChangeIntakeState(m_intake),
        //     new ParallelRaceGroup(driveStraightWithWaypoints(FieldPoint.NOTE1, forAlliance(new Translation2d(3.9, 7.5), m_alliance), FieldPoint.NOTE8, new Rotation2d()), new ChangeIntakeState(m_intake, m_sensors)),
        //     // driveStraight(FieldPoint.NOTE1, FieldPoint.NOTE1),
        //     // new DriveWithProfileNote(noteDetecor::getClosestTranslation2d,m_swerve,new HolonomicDriveController100(),limits, sensor::getFeederSensor, m_intake)
        //     driveStraight( new Pose2d(getTranslation(FieldPoint.NOTE8), new Rotation2d(Math.PI)), new Pose2d(3.4, 6.1, ShooterUtil.getRobotRotationToSpeaker(new Translation2d(3.4, 6.1), kShooterScale))),
        //     new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, -1)
        // );

        return new SequentialCommandGroup(
            // new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, -1),
            driveToStageBase(FieldPoint.STARTSUBWOOFER, FieldPoint.NOTE3),
            
            test(FieldPoint.NOTE3, forAlliance(new Translation2d(1.99, 5.5583), m_alliance), forAlliance(new Translation2d(2.3, 5.5583), m_alliance), FieldPoint.NOTE2),
            
            test(FieldPoint.NOTE2, forAlliance(new Translation2d(1.95, 6.47), m_alliance), forAlliance(new Translation2d(2.307, 6.67), m_alliance), FieldPoint.NOTE1),  
            // new ChangeIntakeState(m_intake),
            driveStraightWithWaypoints(FieldPoint.NOTE1, forAlliance(new Translation2d(3.9, 7.5), m_alliance), FieldPoint.NOTE8, new Rotation2d()),
            // driveStraight(FieldPoint.NOTE1, FieldPoint.NOTE1),
            // new DriveWithProfileNote(noteDetecor::getClosestTranslation2d,m_swerve,new HolonomicDriveController100(),limits, sensor::getFeederSensor, m_intake)
            driveStraight( new Pose2d(getTranslation(FieldPoint.NOTE8), new Rotation2d(Math.PI)), new Pose2d(3.4, 6.1, ShooterUtil.getRobotRotationToSpeaker(new Translation2d(3.4, 6.1), kShooterScale)))
            // new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, -1)
        );
                

    }

    // public Command getTimeoutCommand(Command trajectoryCommand,  ){
        
    // }

    public SequentialCommandGroup wesuck(Alliance alliance, SwerveDriveSubsystem drive, NotePosition24ArrayListener noteDetecor,SwerveKinodynamics limits,SensorInterface sensor){
        return new SequentialCommandGroup(
            new ResetPoseAuto(drive),
            new StowAmpCommand(m_amp),
            new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, 1),
            new ParallelRaceGroup( driveStraight(alliance, FieldPoint.STARTSUBWOOFER, FieldPoint.NOTE2),
                        new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, 1)),
            new ParallelRaceGroup(adjacentWithShooterAngle(alliance, FieldPoint.NOTE2, FieldPoint.NOTE1),
                        new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, 1)),
            new ParallelDeadlineGroup(new WaitCommand(3), new RotateToShooter(m_swerve)),
            new ParallelDeadlineGroup(new WaitCommand(1), new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, 1)),
            driveStraight(alliance, FieldPoint.NOTE1, FieldPoint.DRIVETONOTEHANDOFF),
            new ParallelRaceGroup(new DriveWithProfileNote(noteDetecor::getClosestTranslation2d,m_swerve,new HolonomicDriveController100(),limits, sensor::getFeederSensor, m_intake)), new IntakeSmart(sensor, m_intake));
            
    }

    public TrajectoryCommand100 fiveNoteAuto(Alliance alliance) {
        // return new SequentialCommandGroup(
        //         new StowAmpCommand(m_amp),
        //         new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, true),
        //         new ParallelRaceGroup(driveStraight(FieldPoint.STARTSUBWOOFER, FieldPoint.NOTE3),
        //                 new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
        //         new ParallelRaceGroup(adjacentWithShooterAngle(FieldPoint.NOTE3, FieldPoint.NOTE2),
        //                 new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
        //         new ParallelRaceGroup(adjacentWithShooterAngle(FieldPoint.NOTE2, FieldPoint.NOTE1),
        //                 new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false)),
        //         driveToStraight(FieldPoint.NOTE8),
        //         driveToStraight(FieldPoint.CLOSEWINGSHOT));

        return driveStraight(alliance, FieldPoint.STARTSUBWOOFER, FieldPoint.NOTE3);

            
    }

    public SequentialCommandGroup citrus(Alliance alliance){
        return new SequentialCommandGroup(
            new StowAmpCommand(m_amp),
            new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false),
            driveStraight(alliance, FieldPoint.CITRUSBEGIN, FieldPoint.CITRUSMID),
            driveStraight(alliance, FieldPoint.CITRUSMID, FieldPoint.CITRUSEND)

        );

    }

    public Command tuning() {
        return tuningTrajectory6();
    }
}