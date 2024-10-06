package org.team100.frc2024.motion;

import java.util.List;

import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.commands.ShootPreload;
import org.team100.frc2024.motion.drivetrain.DriveToWithAutoStart;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.intake.ChangeIntakeState;
import org.team100.frc2024.motion.intake.ChangeIntakeState2;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.frc2024.motion.shooter.RampShooter;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.field.FieldPoint2024;
import org.team100.lib.follower.DrivePIDFFollower;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.follower.DriveTrajectoryFollowerFactory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoMaker implements Glassy {
    private static final double kIntakeOffset = 0;

    private final SwerveDriveSubsystem m_swerve;
    private final DriveTrajectoryFollower m_controller;
    private final SensorInterface m_sensors;
    private final List<TimingConstraint> m_slow;
    private final List<TimingConstraint> m_fast;
    private final Intake m_intake;
    private final DrumShooter m_shooter;
    private final FeederSubsystem m_feeder;
    private final double kShooterScale;
    private final LoggerFactory m_logger;
    private final DrivePIDFFollower.Log m_log;
    private final TrajectoryCommand100.Log m_commandLog;
    private final DriveTrajectoryFollowerFactory m_factory;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final TrajectoryVisualization m_viz;

    public AutoMaker(
            LoggerFactory parent,
            SwerveDriveSubsystem swerve,
            DriveTrajectoryFollowerFactory factory,
            DriveTrajectoryFollower controller,
            double shooterScale,
            FeederSubsystem feeder,
            DrumShooter shooter,
            Intake intake,
            SensorInterface sensor,
            SwerveKinodynamics swerveKinodynamics,
            TrajectoryVisualization viz) {
        m_swerve = swerve;
        m_factory = factory;
        m_controller = controller;
        TimingConstraintFactory constraints = new TimingConstraintFactory(swerveKinodynamics);
        m_slow = constraints.allGood();
        m_fast = constraints.fast();
        kShooterScale = shooterScale;
        m_feeder = feeder;
        m_shooter = shooter;
        m_intake = intake;
        m_sensors = sensor;
        m_swerveKinodynamics = swerveKinodynamics;
        m_logger = parent.child(this);
        m_log = new DrivePIDFFollower.Log(m_logger);
        m_commandLog = new TrajectoryCommand100.Log(m_logger);
        m_viz = viz;
    }

    /** This encodes knowledge about how to approach each field point. */
    private Pose2d getPose(Alliance alliance, FieldPoint2024 point) {
        Translation2d translation = FieldPoint2024.getTranslation(alliance, point);
        Rotation2d heading = new Rotation2d(Math.PI);
        switch (point) {
            case NOTE1, NOTE2:
                heading = ShooterUtil.getRobotRotationToSpeaker(alliance, translation, kShooterScale);
                Translation2d offset = new Translation2d(kIntakeOffset * heading.getCos(), kIntakeOffset *
                        heading.getSin());
                return new Pose2d(translation.plus(offset), heading);
            case NOTE3:
                heading = ShooterUtil.getRobotRotationToSpeaker(alliance, translation, kShooterScale);
                Translation2d note3Offset = new Translation2d(.2 * heading.getCos(), .2 *
                        heading.getSin());
                return new Pose2d(translation.plus(note3Offset), heading);
            case CLOSEWINGSHOT, FARWINGSHOT, STAGESHOT, DROPSHOT, COMPLEMENTSHOOT, COMPLEMENTSHOOT2:
                heading = ShooterUtil.getRobotRotationToSpeaker(alliance, translation, kShooterScale);
                return new Pose2d(translation, heading);
            case CENTRALSTAGEOPENING, FARSTAGEOPENING, FARSTAGEADJACENT, DRIVETONOTEHANDOFF:
                return new Pose2d(translation, heading);
            case COMPLEMENTBEGIN:
                heading = Rotation2d.fromDegrees(240);
                return new Pose2d(translation, heading);
            case NOTE4:
                heading = Rotation2d.fromDegrees(180);
                return new Pose2d(translation, heading);
            case NOTE5:
                heading = Rotation2d.fromDegrees(155);
                return new Pose2d(translation, heading);
            default:
                return new Pose2d(translation.plus(new Translation2d(-kIntakeOffset, 0)), heading);
        }
    }

    public TrajectoryCommand100 adjacentWithShooterAngle(Alliance alliance, FieldPoint2024 noteA,
            FieldPoint2024 noteB) {
        Pose2d startPose = getPose(alliance, noteA);
        Pose2d endPose = getPose(alliance, noteB);

        Translation2d translationToGoal = endPose.getTranslation().minus(startPose.getTranslation());
        Translation2d betweenOffset = translationToGoal.times(-.5).plus(new Translation2d(-.875, 0));
        Rotation2d angleToGoal = translationToGoal.getAngle();

        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), angleToGoal.times(1.75));
        Pose2d betweenWaypoint = new Pose2d(endPose.getTranslation().plus(betweenOffset), angleToGoal);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), endPose.getRotation().plus(new Rotation2d(Math.PI)));

        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                betweenWaypoint,
                endWaypoint);

        Rotation2d betweenHeading = new Rotation2d(Math.PI);
        List<Rotation2d> headings = List.of(
                startPose.getRotation(),
                betweenHeading,
                endPose.getRotation());
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory, m_controller, m_viz);
    }

    public TrajectoryCommand100 test(
            Alliance alliance,
            FieldPoint2024 noteA,
            Translation2d waypoint,
            Translation2d waypoint2,
            FieldPoint2024 noteB) {
        Pose2d startPose = getPose(alliance, noteA);
        Pose2d endPose = getPose(alliance, noteB);

        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), Rotation2d.fromDegrees(170));
        // at this point the spline should be parallel to the line between the
        // surrounding points.
        Translation2d toTarget = waypoint2.minus(startWaypoint.getTranslation());
        Pose2d betweenWaypoint = new Pose2d(waypoint, toTarget.getAngle());
        Pose2d midWaypoint = new Pose2d(waypoint2, endPose.getTranslation().minus(waypoint2).getAngle());
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), endPose.getRotation().plus(new Rotation2d(Math.PI)));

        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                betweenWaypoint,
                midWaypoint,
                endWaypoint);

        Rotation2d betweenHeading = new Rotation2d(
                ShooterUtil.getRobotRotationToSpeaker(alliance, waypoint, 0).getRadians());
        List<Rotation2d> headings = List.of(
                startPose.getRotation(),
                betweenHeading,
                betweenHeading,
                endPose.getRotation());
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_slow);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory, m_factory.goodPIDF(m_log),
                m_viz);
    }

    public DriveToWithAutoStart startToNote(Alliance alliance, FieldPoint2024 note) {
        Pose2d endPose = getPose(alliance, note);
        Rotation2d endRotation = new Rotation2d(Math.PI / 2);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), endRotation);
        return new DriveToWithAutoStart(
                m_logger, m_swerve,
                endWaypoint, endPose.getRotation(), m_controller,
                m_swerveKinodynamics, m_viz);
    }

    public TrajectoryCommand100 tuningTrajectory1() {
        List<Pose2d> waypointsM = List.of(
                new Pose2d(2, 2, new Rotation2d()),
                new Pose2d(5, 2, new Rotation2d()));
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d(Math.PI));
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory, m_controller, m_viz);
    }

    public TrajectoryCommand100 tuningTrajectory6() {
        List<Pose2d> waypointsM = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                new Pose2d(1, 1, Rotation2d.fromDegrees(45)));
        List<Rotation2d> headings = List.of(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory, m_controller, m_viz);
    }

    public TrajectoryCommand100 tuningTrajectory2() {
        List<Pose2d> waypointsM = List.of(
                new Pose2d(5, 2, new Rotation2d(Math.PI)),
                new Pose2d(2, 2, new Rotation2d(Math.PI)));
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d(Math.PI));
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory, m_controller, m_viz);
    }

    public TrajectoryCommand100 tuningTrajectory3() {
        List<Pose2d> waypointsM = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(Math.PI), new Rotation2d());
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory, m_controller, m_viz);
    }

    public TrajectoryCommand100 tuningTrajectory4() {
        List<Pose2d> waypointsM = List.of(new Pose2d(), new Pose2d());
        List<Rotation2d> headings = List.of(new Rotation2d(), new Rotation2d(Math.PI));
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory, m_controller, m_viz);
    }

    public TrajectoryCommand100 driveToStageBase(Alliance alliance, FieldPoint2024 start, FieldPoint2024 end) {
        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);
        Rotation2d angleToGoal = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();

        Pose2d waypoint = new Pose2d(FieldPoint2024.forAlliance(new Translation2d(1.92, 4.84), alliance), angleToGoal);
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(),
                waypoint.getTranslation().minus(startPose.getTranslation()).getAngle());
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);

        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                waypoint,
                endWaypoint);

        List<Rotation2d> headings = List.of(
                startPose.getRotation(),
                endPose.getRotation(),
                endPose.getRotation());
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory,
                m_factory.stageBase(m_log), m_viz);
    }

    public TrajectoryCommand100 throughStage(Alliance alliance, FieldPoint2024 start, FieldPoint2024 end) {

        Translation2d point = new Translation2d(5.0, 4.2);
        point = FieldPoint2024.forRedAlliance(point, alliance);

        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);
        Rotation2d angleToGoal = endPose.getTranslation().minus(point).getAngle();

        Pose2d waypoint = new Pose2d(point, angleToGoal);
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(),
                waypoint.getTranslation().minus(startPose.getTranslation()).getAngle());
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);

        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                waypoint,
                endWaypoint);

        List<Rotation2d> headings = List.of(
                startPose.getRotation(),
                endPose.getRotation(),
                endPose.getRotation());
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory,
                m_factory.complementPIDF(m_log), m_viz);
    }

    public TrajectoryCommand100 aroundStage(Alliance alliance, FieldPoint2024 start, FieldPoint2024 end) {
        Translation2d point = (new Translation2d(5.7, 6.4));
        point = FieldPoint2024.forRedAlliance(point, alliance);
        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);
        Rotation2d angleToGoal = endPose.getTranslation().minus(point).getAngle();

        Pose2d waypoint = new Pose2d(point, angleToGoal);
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(),
                waypoint.getTranslation().minus(startPose.getTranslation()).getAngle());
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);

        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                waypoint,
                endWaypoint);
        List<Rotation2d> headings = List.of(
                startPose.getRotation(),
                endPose.getRotation(),
                endPose.getRotation());
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_slow);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory,
                m_factory.complementPIDF(m_log), m_viz);
    }

    public TrajectoryCommand100 aroundStage(Alliance alliance, FieldPoint2024 start, FieldPoint2024 end,
            Rotation2d heading) {
        Translation2d point = (new Translation2d(5.7, 6.4));
        point = FieldPoint2024.forRedAlliance(point, alliance);
        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);
        Rotation2d angleToGoal = endPose.getTranslation().minus(point).getAngle();

        Pose2d waypoint = new Pose2d(point, angleToGoal);
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(),
                waypoint.getTranslation().minus(startPose.getTranslation()).getAngle());
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);

        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                waypoint,
                endWaypoint);
        List<Rotation2d> headings = List.of(
                startPose.getRotation(),
                heading,
                heading);
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_slow);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory,
                m_factory.complementPIDF(m_log), m_viz);
    }

    public TrajectoryCommand100 aroundStage(Alliance alliance, FieldPoint2024 start, FieldPoint2024 end,
            double begHeading) {
        Translation2d point = (new Translation2d(5.7, 6.4));
        point = FieldPoint2024.forRedAlliance(point, alliance);
        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);
        Rotation2d angleToGoal = endPose.getTranslation().minus(point).getAngle();

        Pose2d waypoint = new Pose2d(point, angleToGoal);
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(),
                waypoint.getTranslation().minus(startPose.getTranslation()).getAngle());
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);

        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                waypoint,
                endWaypoint);
        List<Rotation2d> headings = List.of(
                new Rotation2d(begHeading),
                endPose.getRotation(),
                endPose.getRotation());
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_slow);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory,
                m_factory.complementPIDF(m_log), m_viz);
    }

    public TrajectoryCommand100 driveStraight(Alliance alliance, FieldPoint2024 start, FieldPoint2024 end) {
        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);

        Rotation2d angleToGoal = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), angleToGoal);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);

        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                endWaypoint);
        List<Rotation2d> headings = List.of(
                startPose.getRotation(),
                endPose.getRotation());
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory,
                m_factory.straightPIDF(m_log), m_viz);
    }

    public TrajectoryCommand100 driveStraight(
            Alliance alliance,
            FieldPoint2024 start,
            FieldPoint2024 end,
            Rotation2d begHeading,
            Rotation2d endHeading) {
        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);

        Rotation2d angleToGoal = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), angleToGoal);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);

        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(begHeading, endHeading);
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory,
                m_factory.straightPIDF(m_log), m_viz);
    }

    public TrajectoryCommand100 driveStraight(
            Alliance alliance,
            FieldPoint2024 start,
            FieldPoint2024 end,
            double splineStartDirection,
            double endingSplineDirection) {
        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);

        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), new Rotation2d(splineStartDirection));
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), new Rotation2d(endingSplineDirection));

        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(startPose.getRotation(), endPose.getRotation());
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory,
                m_factory.complementPIDF(m_log), m_viz);
    }

    public TrajectoryCommand100 driveStraight(Pose2d start, Pose2d end, Alliance alliance) {

        Pose2d startPose = start;

        Translation2d correctTranslation = FieldPoint2024.forAlliance(end.getTranslation(), alliance);
        Pose2d endPose = new Pose2d(FieldPoint2024.forAlliance(end.getTranslation(), alliance),
                ShooterUtil.getRobotRotationToSpeaker(alliance, correctTranslation, kShooterScale));

        Rotation2d angleToGoal = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();

        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), angleToGoal);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);

        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);
        List<Rotation2d> headings = List.of(startPose.getRotation(), endPose.getRotation());
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(
                m_commandLog,
                m_swerve,
                trajectory,
                m_factory.straightPIDF(m_log), m_viz);
    }

    public TrajectoryCommand100 driveStraightWithWaypoints(
            Alliance alliance,
            FieldPoint2024 start,
            FieldPoint2024 end) {
        Pose2d startPose = getPose(alliance, start);
        Pose2d endPose = getPose(alliance, end);

        Rotation2d angleToGoal = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();

        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), angleToGoal);
        Pose2d endWaypoint = new Pose2d(endPose.getTranslation(), angleToGoal);
        List<Pose2d> waypointsM = List.of(startWaypoint, endWaypoint);

        Rotation2d startHeading = startPose.getRotation();
        Rotation2d endHeading = new Rotation2d(Math.PI);
        List<Rotation2d> headings = List.of(startHeading, endHeading);
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory,
                m_factory.newNewPIDF(m_log), m_viz);
    }

    public TrajectoryCommand100 stageManeuver(Alliance alliance, FieldPoint2024 start, FieldPoint2024 between,
            FieldPoint2024 end) {
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

        List<Pose2d> waypointsM = List.of(
                startWaypoint,
                betweenWaypoint,
                endWaypoint);
        List<Rotation2d> headings = List.of(
                startPose.getRotation(),
                betweenPose.getRotation(),
                endPose.getRotation());
        Trajectory100 trajectory = TrajectoryPlanner.restToRest(waypointsM, headings, m_fast);
        return new TrajectoryCommand100(m_commandLog, m_swerve, trajectory, m_controller, m_viz);
    }

    public TrajectoryCommand100 throughCentralStageOpening(Alliance alliance, FieldPoint2024 start,
            FieldPoint2024 end) {
        return stageManeuver(alliance, start, FieldPoint2024.CENTRALSTAGEOPENING, end);
    }

    public TrajectoryCommand100 throughFarStageOpening(Alliance alliance, FieldPoint2024 start, FieldPoint2024 end) {
        return stageManeuver(alliance, start, FieldPoint2024.FARSTAGEOPENING, end);
    }

    public TrajectoryCommand100 aroundStageClose(Alliance alliance, FieldPoint2024 start, FieldPoint2024 end) {
        return stageManeuver(alliance, start, FieldPoint2024.CLOSESTAGEADJACENT, end);
    }

    public TrajectoryCommand100 aroundStageFar(Alliance alliance, FieldPoint2024 start, FieldPoint2024 end) {
        return stageManeuver(alliance, start, FieldPoint2024.FARSTAGEADJACENT, end);
    }

    public DriveToWaypoint100 driveToStraight(Alliance alliance, FieldPoint2024 point) {
        return new DriveToWaypoint100(m_logger, getPose(alliance, point), m_swerve, m_controller, m_swerveKinodynamics,
                1, m_viz);
    }

    public SequentialCommandGroup eightNoteAuto(Alliance alliance) {
        return new SequentialCommandGroup(startToNote(alliance, FieldPoint2024.NOTE3),
                adjacentWithShooterAngle(alliance, FieldPoint2024.NOTE3, FieldPoint2024.NOTE2),
                adjacentWithShooterAngle(alliance, FieldPoint2024.NOTE2, FieldPoint2024.NOTE1),
                driveToStraight(alliance, FieldPoint2024.NOTE8),
                driveToStraight(alliance, FieldPoint2024.CLOSEWINGSHOT),
                aroundStageClose(alliance, FieldPoint2024.CLOSEWINGSHOT, FieldPoint2024.NOTE7),
                throughCentralStageOpening(alliance, FieldPoint2024.NOTE7, FieldPoint2024.STAGESHOT),
                throughCentralStageOpening(alliance, FieldPoint2024.STAGESHOT, FieldPoint2024.NOTE6),
                throughCentralStageOpening(alliance, FieldPoint2024.NOTE6, FieldPoint2024.STAGESHOT),
                throughCentralStageOpening(alliance, FieldPoint2024.NOTE5, FieldPoint2024.STAGESHOT),
                throughCentralStageOpening(alliance, FieldPoint2024.STAGESHOT, FieldPoint2024.NOTE5));
    }

    public SequentialCommandGroup complementAuto(Alliance alliance) {
        return new SequentialCommandGroup(driveToStraight(alliance, FieldPoint2024.NOTE4),
                driveToStraight(alliance, FieldPoint2024.FARWINGSHOT),
                aroundStageFar(alliance, FieldPoint2024.FARWINGSHOT, FieldPoint2024.NOTE5),
                throughCentralStageOpening(alliance, FieldPoint2024.NOTE5, FieldPoint2024.STAGESHOT),
                throughFarStageOpening(alliance, FieldPoint2024.STAGESHOT, FieldPoint2024.DROPSHOT));
    }

    public SequentialCommandGroup fourNoteAuto(Alliance alliance,
            SensorInterface sensor) {

        return new SequentialCommandGroup(
                new PrintCommand("FOUR NOTE AUTO"),
                new ShootPreload(m_logger, sensor, m_shooter, m_intake, m_feeder, m_swerve, true),
                new ParallelRaceGroup(driveToStageBase(alliance, FieldPoint2024.STARTSUBWOOFER, FieldPoint2024.NOTE3),
                        new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, false)),
                new ParallelDeadlineGroup(
                        test(alliance,
                                FieldPoint2024.NOTE3,
                                FieldPoint2024.forAlliance(new Translation2d(1.99, 5.5583), alliance),
                                FieldPoint2024.forAlliance(new Translation2d(2.3, 5.5583), alliance),
                                FieldPoint2024.NOTE2),
                        new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, false)),
                new ParallelDeadlineGroup(
                        test(alliance,
                                FieldPoint2024.NOTE2,
                                FieldPoint2024.forAlliance(new Translation2d(1.95, 6.47), alliance),
                                FieldPoint2024.forAlliance(new Translation2d(2.307, 6.67), alliance),
                                FieldPoint2024.NOTE1),
                        new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, false)),
                new ParallelDeadlineGroup(
                        driveStraightWithWaypoints(
                                alliance,
                                FieldPoint2024.NOTE1,
                                FieldPoint2024.NOTE8),
                        new ChangeIntakeState2(m_intake, m_sensors)),
                new ParallelRaceGroup(
                        driveStraight(
                                new Pose2d(FieldPoint2024.getTranslation(alliance, FieldPoint2024.NOTE8),
                                        new Rotation2d(Math.PI)),
                                new Pose2d(3.4, 6.1,
                                        ShooterUtil.getRobotRotationToSpeaker(alliance, new Translation2d(3.4, 6.1),
                                                kShooterScale)),
                                alliance),
                        new RampShooter(m_shooter)),
                new ShootSmart(sensor, m_shooter, m_intake, m_feeder, m_swerve, false));
    }

    public Command sibling(Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                            driveStraight(alliance, FieldPoint2024.COMPLEMENTBEGIN, FieldPoint2024.NOTE4,
                                    -Math.toRadians(70), 0),
                            new ChangeIntakeState(m_intake, m_sensors)),
                    driveStraight(alliance, FieldPoint2024.NOTE4, FieldPoint2024.NOTE8, 3.5, 3.5));
        } else {
            return new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                            driveStraight(alliance, FieldPoint2024.COMPLEMENTBEGIN, FieldPoint2024.NOTE4,
                                    Math.toRadians(70), 0),
                            new ChangeIntakeState(m_intake, m_sensors)),
                    driveStraight(alliance, FieldPoint2024.NOTE4, FieldPoint2024.NOTE8, 4, 4));
        }
    }

    // all this extra printing is to troubleshoot the simulator
    public SequentialCommandGroup citrus(Alliance alliance) {
        if (alliance == Alliance.Red) {
            return new SequentialCommandGroup(
                    new PrintCommand("red citrus 1"),
                    new ShootPreload(m_logger, m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false),
                    new PrintCommand("red citrus 2"),
                    new ParallelDeadlineGroup(
                            driveStraight(alliance, FieldPoint2024.COMPLEMENTBEGIN, FieldPoint2024.NOTE4, Math.PI / 4,
                                    0),
                            new ChangeIntakeState(m_intake, m_sensors)),
                    new PrintCommand("red citrus 3"),
                    new ParallelRaceGroup(driveStraight(alliance, FieldPoint2024.NOTE4, FieldPoint2024.COMPLEMENTSHOOT,
                            Math.toRadians(180), Math.toRadians(245)), new RampShooter(m_shooter)),
                    new PrintCommand("red citrus 4"),
                    new ParallelRaceGroup(
                            new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false),
                            new WaitCommand(1.5)),
                    new PrintCommand("red citrus 5"),
                    new ParallelDeadlineGroup(
                            throughStage(alliance, FieldPoint2024.COMPLEMENTSHOOT, FieldPoint2024.NOTE6),
                            new ChangeIntakeState(m_intake, m_sensors)),
                    new PrintCommand("red citrus 6"),
                    new ParallelRaceGroup(
                            driveStraight(alliance, FieldPoint2024.NOTE6, FieldPoint2024.COMPLEMENTSHOOT2, Math.PI,
                                    Math.toRadians(-135)),
                            new RampShooter(m_shooter)),
                    new PrintCommand("red citrus 7"),
                    new ParallelRaceGroup(
                            new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false),
                            new WaitCommand(1.5)));
        } else {
            return new SequentialCommandGroup(
                    new PrintCommand("blue citrus 1"),
                    new ShootPreload(m_logger, m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false),
                    new PrintCommand("blue citrus 2"),
                    new ParallelDeadlineGroup(
                            driveStraight(alliance, FieldPoint2024.COMPLEMENTBEGIN, FieldPoint2024.NOTE4,
                                    3 * Math.PI / 2, 0),
                            new ChangeIntakeState(m_intake, m_sensors)),
                    new PrintCommand("blue citrus 3"),
                    new ParallelRaceGroup(driveStraight(alliance, FieldPoint2024.NOTE4, FieldPoint2024.COMPLEMENTSHOOT,
                            Math.toRadians(180), Math.toRadians(245.0 - 180)),
                            new RampShooter(m_shooter)),
                    new PrintCommand("blue citrus 4"),
                    new ParallelRaceGroup(
                            new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false),
                            new WaitCommand(1.5)),
                    new PrintCommand("blue citrus 5"),
                    new ParallelDeadlineGroup(
                            throughStage(alliance, FieldPoint2024.COMPLEMENTSHOOT, FieldPoint2024.NOTE6),
                            new ChangeIntakeState(m_intake, m_sensors)),
                    new PrintCommand("blue citrus 6"),
                    new ParallelRaceGroup(
                            driveStraight(alliance, FieldPoint2024.NOTE6, FieldPoint2024.COMPLEMENTSHOOT2, Math.PI,
                                    Math.toRadians(140)),
                            new RampShooter(m_shooter)),
                    new PrintCommand("blue citrus 7"),
                    new ParallelRaceGroup(
                            new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false),
                            new WaitCommand(1.5)));
        }
    }

    public SequentialCommandGroup citrusv2(Alliance alliance) {
        if (alliance == Alliance.Red) {
            return new SequentialCommandGroup(
                    new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false),
                    new ParallelRaceGroup(
                            driveStraight(alliance, FieldPoint2024.COMPLEMENTBEGIN, FieldPoint2024.NOTE4, Math.PI / 4,
                                    0),
                            new ChangeIntakeState(m_intake, m_sensors)),
                    new ParallelRaceGroup(
                            driveStraight(alliance, FieldPoint2024.NOTE4, FieldPoint2024.COMPLEMENTSHOOT,
                                    Math.toRadians(180),
                                    Math.toRadians(245)),
                            new RampShooter(m_shooter)),
                    new ShootPreload(m_logger, m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false),
                    new ParallelRaceGroup(
                            aroundStage(alliance, FieldPoint2024.COMPLEMENTSHOOT, FieldPoint2024.NOTE5),
                            new ChangeIntakeState(m_intake, m_sensors)),
                    new ParallelRaceGroup(
                            aroundStage(alliance, FieldPoint2024.NOTE5, FieldPoint2024.COMPLEMENTSHOOT),
                            new RampShooter(m_shooter)),
                    new ShootPreload(m_logger, m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false));
        } else {
            return new SequentialCommandGroup(
                    new ShootSmart(m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false),
                    new ParallelRaceGroup(
                            driveStraight(alliance, FieldPoint2024.COMPLEMENTBEGIN, FieldPoint2024.NOTE4,
                                    3 * Math.PI / 2, 0),
                            new ChangeIntakeState(m_intake, m_sensors)),
                    new ParallelRaceGroup(
                            driveStraight(alliance, FieldPoint2024.NOTE4, FieldPoint2024.COMPLEMENTSHOOT,
                                    Math.toRadians(180),
                                    Math.toRadians(245.0 - 180)),
                            new RampShooter(m_shooter)),
                    new ShootPreload(m_logger, m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false),
                    new ParallelRaceGroup(
                            aroundStage(alliance, FieldPoint2024.COMPLEMENTSHOOT, FieldPoint2024.NOTE5,
                                    Rotation2d.fromDegrees(200)),
                            new ChangeIntakeState(m_intake, m_sensors)),
                    new ParallelRaceGroup(
                            aroundStage(alliance, FieldPoint2024.NOTE5, FieldPoint2024.COMPLEMENTSHOOT,
                                    Math.toRadians(200)),
                            new RampShooter(m_shooter)),
                    new ShootPreload(m_logger, m_sensors, m_shooter, m_intake, m_feeder, m_swerve, false));
        }
    }

    public Command tuning() {
        return tuningTrajectory6();
    }

}