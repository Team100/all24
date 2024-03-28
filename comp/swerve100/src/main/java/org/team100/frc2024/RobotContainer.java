package org.team100.frc2024;

import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.RobotState100.AmpState100;
import org.team100.frc2024.RobotState100.FeederState100;
import org.team100.frc2024.RobotState100.IntakeState100;
import org.team100.frc2024.RobotState100.ShooterState100;
import org.team100.frc2024.commands.AutonCommand;
import org.team100.frc2024.commands.drivetrain.DriveWithProfileNote;
import org.team100.frc2024.config.AutonChooser;
import org.team100.frc2024.motion.AutoMaker;
import org.team100.frc2024.motion.ChangeAmpState;
import org.team100.frc2024.motion.FeedCommand;
import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.OuttakeCommand;
import org.team100.frc2024.motion.amp.AmpDefault;
import org.team100.frc2024.motion.amp.AmpSubsystem;
import org.team100.frc2024.motion.amp.OuttakeAmp;
import org.team100.frc2024.motion.amp.PivotAmp;
import org.team100.frc2024.motion.climber.ClimberDefault;
import org.team100.frc2024.motion.climber.ClimberPosition;
import org.team100.frc2024.motion.climber.ClimberSubsystem;
import org.team100.frc2024.motion.drivetrain.manual.AmpLockCommand;
import org.team100.frc2024.motion.drivetrain.manual.ManualWithAmpLock;
import org.team100.frc2024.motion.drivetrain.manual.ManualWithShooterLock;
import org.team100.frc2024.motion.drivetrain.manual.ShooterLockCommand;
import org.team100.frc2024.motion.intake.FeederDefault;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.intake.IntakeDefault;
import org.team100.frc2024.motion.intake.IntakeFactory;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.frc2024.motion.shooter.SetDefaultShoot;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.frc2024.motion.shooter.ShooterDefault;
import org.team100.lib.SubsystemPriority;
import org.team100.lib.SubsystemPriority.Priority;
import org.team100.lib.commands.AllianceCommand;
import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.DriveToWaypoint3;
import org.team100.lib.commands.drivetrain.FancyTrajectory;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.Rotate;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DriveMotionControllerFactory;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.copies.SwerveDrivePoseEstimator100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.localization.VisionDataProvider24;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.manual.FieldManualWithNoteRotation;
import org.team100.lib.motion.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualWithHeading;
import org.team100.lib.motion.drivetrain.manual.ManualWithNoteRotation;
import org.team100.lib.motion.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.motion.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.HeadingFactory;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.util.Names;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Try to keep this container clean; if there's something you want to keep but
 * don't need right now, cut and paste it into {@link RobotContainerParkingLot}.
 */
public class RobotContainer implements Glassy {
    private static final double kDriveCurrentLimit = 60;
    private final Telemetry t = Telemetry.get();

    private final HeadingInterface m_heading;
    private final LEDIndicator m_indicator;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    final SwerveDriveSubsystem m_drive;
    private final SwerveModuleCollection m_modules;

    private final Command m_auton;

    private final SelfTestRunner m_selfTest;

    // final IndexerSubsystem m_indexer;
    final AmpSubsystem m_amp;
    private final ClimberSubsystem m_climber;
    private final Shooter m_shooter;
    final Intake m_intake;
    private final LEDSubsystem m_ledSubsystem;
    private final SensorInterface m_sensors;
    private final FeederSubsystem m_feeder;

    private final DriverControl driverControl;
    private final OperatorControl operatorControl;
    // Commands
    private final PivotAmp m_pivotAmp;

    private final String m_name;

    public RobotContainer(TimedRobot robot) throws IOException {
        m_name = Names.name(this);

        driverControl = new DriverControlProxy();
        operatorControl = new OperatorControlProxy();
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();

        m_layout = new AprilTagFieldLayoutWithCorrectOrientation();

        switch (Identity.instance) {
            case COMP_BOT:
                m_sensors = new CompSensors(2, 1, 4); // Definitely real numbers
                break;
            default:
                // always returns false
                m_sensors = new MockSensors();
        }

        m_modules = SwerveModuleCollection.get(kDriveCurrentLimit, swerveKinodynamics);

        m_heading = HeadingFactory.get(swerveKinodynamics, m_modules);

        // ignores the rotation derived from vision.
        SwerveDrivePoseEstimator100 poseEstimator = swerveKinodynamics.newPoseEstimator(
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE)); // 0.1 0.1

        VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                m_layout,
                poseEstimator,
                poseEstimator::getSampledRotation);
        visionDataProvider.enable();

        NotePosition24ArrayListener notePositionDetector = new NotePosition24ArrayListener(poseEstimator);
        notePositionDetector.enable();

        SwerveLocal swerveLocal = new SwerveLocal(swerveKinodynamics, m_modules);

        m_drive = new SwerveDriveSubsystem(
                m_heading,
                poseEstimator,
                swerveLocal,
                driverControl::speed);

        m_feeder = new FeederSubsystem(39);

        m_intake = IntakeFactory.get(m_sensors);

        // @sanjan 3/25
        // LEDStrip strip1 = new LEDStrip(LEDGroup.ONE, 160, 0);
        // m_indicator = new LEDIndicator(0, StripFactory.get());

        m_indicator = new LEDIndicator(0);

        m_shooter = new DrumShooter(3, 13, 7, 58);

        m_ledSubsystem = new LEDSubsystem(
                m_indicator,
                m_sensors,
                m_shooter,
                visionDataProvider);

        // / = new IndexerSubsystem(63); // NEED CAN FOR AMP MOTOR //5
        m_amp = new AmpSubsystem(2);
        m_pivotAmp = new PivotAmp(m_amp, operatorControl::ampPosition);

        m_climber = new ClimberSubsystem(60, 61);

        // show mode locks slow speed.

        ////////////////////////////
        //
        // DRIVETRAIN COMMANDS
        //

        // joel 3/15/24 removed these, i don't think we use them.
        // whileTrue(driverControl::defense, m_drive.runInit(m_drive::defense));
        // whileTrue(driverControl::steer0, m_drive.runInit(m_drive::steer0));
        // whileTrue(driverControl::steer90, m_drive.runInit(m_drive::steer90));

        // RESET ZERO
        // on xbox this is "back"
        onTrue(driverControl::resetRotation0, new ResetPose(m_drive, 0, 0, 0));

        // RESET 180
        // on xbox this is "start"
        onTrue(driverControl::resetRotation180, new SetRotation(m_drive, GeometryUtil.kRotation180));

        // joel 3/15/24 removed ResetPose
        // onTrue(driverControl::resetRotation0, new ResetPose(m_drive, 0, 0, 0));
        // on xbox this is left bumper
        // on joystick this is button 4
        // on starting zone line lined up with note
        // joel mar 15 turned this off, i think it's the cause of the "spontaneous gyro
        // reset" the drivers experience
        // onTrue(driverControl::resetPose, new ResetPose(m_drive, .5, 7, 0));

        HolonomicDriveController3 controller = new HolonomicDriveController3();

        HolonomicDriveController100 dthetaController = new HolonomicDriveController100();

        whileTrue(driverControl::rotate0, new Rotate(m_drive, m_heading, swerveKinodynamics, 0));

        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();

        TrajectoryPlanner planner = new TrajectoryPlanner();

        whileTrue(driverControl::driveWithFancyTrajec,
                new FancyTrajectory(m_drive, planner, constraints));

        // playing with trajectory followers
        TrajectoryConfig config = new TrajectoryConfig(5, 5);
        StraightLineTrajectory maker = new StraightLineTrajectory(config);

        // field center, roughly, facing to the left.
        Pose2d goal = new Pose2d(1.877866, 7.749999, GeometryUtil.kRotation90);
        Command follower = new DriveToWaypoint3(goal, m_drive, maker, controller);
        // whileTrue(driverControl::test, follower);

        // 254 PID follower
        DriveMotionController drivePID = DriveMotionControllerFactory.goodPIDF();

        // Drive With Profile
        whileTrue(driverControl::driveToNote,
                new DriveWithProfileNote(notePositionDetector::getClosestTranslation2d, m_drive, dthetaController,
                        swerveKinodynamics, () -> !m_sensors.getIntakeSensor(), m_intake));

        // whileTrue(driverControl::test, new Amp(m_drive::getPose, m_drive, planner,
        // drivePID, swerveKinodynamics));

        // whileTrue(driverControl::test, new DriveWithTrajectory(m_drive, planner,
        // drivePP, swerveKinodynamics, "src/main/deploy/choreo/crossField.traj"));

        SwerveState testState = new SwerveState(
                new State100(1, -3),
                new State100(1, -1), null);
        // RunCommand run = new RunCommand(() -> ShooterUtil.getAngleWhileMoving(10, 7,
        // testState), m_shooter);

        // whileTrue(driverControl::test, run);

        whileTrue(operatorControl::intake,
                new StartEndCommand(() -> RobotState100.changeIntakeState(IntakeState100.INTAKE),
                        () -> RobotState100.changeIntakeState(IntakeState100.STOP)));

        whileTrue(operatorControl::outtake,
                new OuttakeCommand(m_intake, m_shooter, m_amp, m_feeder));

        whileTrue(operatorControl::ramp,
                new SetDefaultShoot(m_shooter, ShooterState100.DEFAULTSHOOT));

        // whileTrue(operatorControl::ramp,
        // new ClimberPosition(m_climber));

        whileTrue(operatorControl::feed, new StartEndCommand(() -> RobotState100.changeFeederState(FeederState100.FEED),
                () -> RobotState100.changeFeederState(FeederState100.STOP)));

        // whileTrue(operatorControl::pivotToAmpPosition,
        // new ChangeAmpState(AmpState100.UP, m_amp));

        whileTrue(operatorControl::pivotToAmpPosition, new ChangeAmpState(AmpState100.UP, m_amp));

        // whileTrue(operatorControl::pivotToDownPosition, new Test2());

        whileTrue(operatorControl::pivotToDownPosition, new SetDefaultShoot(m_shooter, ShooterState100.DEFAULTSHOOT));

        whileTrue(operatorControl::feedToAmp, new FeedCommand(m_intake, m_shooter, m_amp, m_feeder, m_sensors));

        whileTrue(operatorControl::rezero, new SetDefaultShoot(m_shooter, ShooterState100.TEST));

        whileTrue(operatorControl::outtakeFromAmp, new OuttakeAmp());

        // TODO: spin up the shooter whenever the robot is in range.

        // whileTrue(operatorControl::ramp, new Ramp());

        // whileTrue(operatorControl::index, m_indexer.run(m_indexer::index));
        // whileTrue(operatorControl::index, new IndexCommand(m_indexer, () -> true));
        // operatorControl.index().whileTrue(new IndexCommand(m_indexer, () ->
        // (m_amp.inPosition())));
        // operatorControl.index().whileTrue(new IndexCommand(m_indexer, () ->
        // (!m_intake.noteInIntake())));

        // TODO: presets
        // m_climber.setDefaultCommand(m_climber.run(() ->
        // m_climber.set(operatorControl.climberState())));

        // whileTrue(driverControl::test,
        // CommandMaker.choreo(Choreo.getTrajectory("curvyField"), m_drive));

        ///////////////////////////
        //
        // DRIVE
        //

        PIDController thetaController = new PIDController(4, 0, 0); // 1.7
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController omegaController = new PIDController(0, 0, 0); // .5
        PIDController omega2Controller = new PIDController(0, 0, 0); // .5

        DriveManually driveManually = new DriveManually(driverControl::twist, m_drive);

        driveManually.register("MODULE_STATE", false,
                new SimpleManualModuleStates(m_name, swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(m_name, swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_FACING_NOTE", false,
                new ManualWithNoteRotation(
                        m_name,
                        swerveKinodynamics,
                        m_heading,
                        notePositionDetector::getClosestTranslation2d,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(m_name, swerveKinodynamics));

        driveManually.register("SNAPS", true,
                new ManualWithHeading(
                        m_name,
                        swerveKinodynamics,
                        m_heading,
                        driverControl::desiredRotation,
                        thetaController,
                        omegaController));
        driveManually.register("FIELD_RELATIVE_FACING_NOTE", false,
                new FieldManualWithNoteRotation(
                        m_name,
                        swerveKinodynamics,
                        m_heading,
                        notePositionDetector::getClosestTranslation2d,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("LOCKED", false,
                new ManualWithTargetLock(
                        m_name,
                        swerveKinodynamics,
                        m_heading,
                        driverControl::target,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("SHOOTER_LOCK", false,
                new ManualWithShooterLock(
                        m_name,
                        swerveKinodynamics,
                        m_heading,
                        thetaController,
                        omegaController,
                        0.25));

        // ManualWithShooterLock shooterLock = new ManualWithShooterLock(
        // m_name,
        // swerveKinodynamics,
        // m_heading,
        // thetaController,
        // omegaController,
        // 0.25);

        // whileTrue(driverControl::test, new PrimitiveAuto(m_drive, shooterLock,
        // planner, drivePID, drivePP, swerveKinodynamics, m_heading));

        // whileTrue(driverControl::test, Commands.startEnd(() ->
        // RobotState100.changeIntakeState(IntakeState100.INTAKE),
        // () -> RobotState100.changeIntakeState(IntakeState100.STOP)));
        // whileTrue(driverControl::driveToAmp, new DriveWithProfile2(() -> new
        // Pose2d(1.834296, 7.474794, new Rotation2d(Math.PI / 2)), m_drive,
        // new HolonomicDriveController100(), swerveKinodynamics));
        // whileTrue(driverControl::test, new DriveToAmp(m_drive, swerveKinodynamics,
        // planner, drivePID, m_alliance));

        ManualWithShooterLock shooterLock = new ManualWithShooterLock(
                m_name,
                swerveKinodynamics,
                m_heading,
                thetaController,
                omega2Controller,
                0.25);

                ManualWithAmpLock ampLock = new ManualWithAmpLock(
                    m_name,
                    swerveKinodynamics,
                    m_heading,
                    thetaController,
                    omega2Controller);

        // whileTrue(driverControl::shooterLock, new ShooterLockCommand(shooterLock,
        // driverControl::twist, m_drive));

        // whileTrue(driverControl::test, new ShooterLockCommand(shooterLock,
        // driverControl::twist, m_drive));

        // whileTrue(driverControl::test, new DriveToState101(new Pose2d(15.446963,
        // 1.522998, Rotation2d.fromDegrees(-60)), new Twist2d(0, 0, 0), m_drive,
        // planner, drivePID, swerveKinodynamics));

        AutoMaker m_AutoMaker = new AutoMaker(
                m_drive,
                planner,
                drivePID,
                swerveKinodynamics,
                0,
                m_feeder,
                m_shooter,
                m_intake,
                m_sensors,
                notePositionDetector,
                m_amp,
                m_heading,
                constraints);

        // whileTrue(driverControl::circle, m_AutoMaker.fiveNoteAuto());
        // whileTrue(driverControl::shooterLock, new ShooterLockCommand(shooterLock,
        // driverControl::twist, m_drive));
        // whileTrue(driverControl::shooterLock,m_AutoMaker.fourNoteAuto(Alliance.Red, swerveKinodynamics, m_sensors));

        // whileTrue(driverControl::shooterLock,
        //         new ManualWithShooterLock(m_name, swerveKinodynamics, m_heading, thetaController, omegaController,
        //                 kDriveCurrentLimit));

        whileTrue(driverControl::ampLock,
                        new AmpLockCommand(ampLock, driverControl::twist, m_drive));

        // whileTrue(driverControl::shooterLock,
        // new AllianceCommand(m_AutoMaker.tuningTrajectory6(),
        // m_AutoMaker.tuningTrajectory6()));

        // whileTrue(driverControl::test, new DriveToState101(new Pose2d(15.446963,
        // 1.522998, Rotation2d.fromDegrees(-60)), new Twist2d(0, 0, 0), m_drive,
        // planner, drivePID, swerveKinodynamics));
        // AutoMaker m_AutoMaker = new AutoMaker(m_drive, planner, drivePID,
        // swerveKinodynamics, 0, m_alliance);

        // on a roborio 1 this takes 0.2 sec, so 10 cycles. less than 0.8 but still a
        // lot.
        whileTrue(driverControl::shooterLock,
                new ShooterLockCommand(shooterLock, driverControl::twist, m_drive));

        whileTrue(driverControl::test,
                new AmpLockCommand(ampLock, driverControl::twist, m_drive));

        // whileTrue(driverControl::shooterLock,
        //         new ClimberPosition(m_climber));
        // whileTrue(driverControl::shooterLock, new ShootSmart(m_sensors, m_shooter,
        // m_intake, m_feeder, m_drive));

        // AutoMaker m_AutoMaker = new AutoMaker(m_drive, planner, drivePID,
        // swerveKinodynamics, 0, m_alliance);
        // whileTrue(driverControl::shooterLock, m_AutoMaker.eightNoteAuto());

        // whileTrue(driverControl::test, new DriveToState101(new Pose2d(15.446963,
        // 1.522998, Rotation2d.fromDegrees(-60)), new Twist2d(0, 0, 0), m_drive,
        // planner, drivePID, swerveKinodynamics));
        // AutoMaker m_AutoMaker = new AutoMaker(m_drive, planner, drivePID,
        // swerveKinodynamics, 0, m_alliance, m_shooter, m_feeder);
        // whileTrue(driverControl::test, m_AutoMaker.eightNoteAuto());

        // whileTrue(driverControl::test, new PrimitiveAuto(m_drive, shooterLock,
        // planner, drivePID, drivePP, swerveKinodynamics, m_heading));

        m_drive.setDefaultCommand(driveManually);

        SubsystemPriority.addSubsystem(m_drive, driveManually, Priority.ONE);
        SubsystemPriority.addSubsystem(m_shooter,
                new ShooterDefault(m_shooter, m_drive, operatorControl::pivotUp, operatorControl::pivotDown),
                Priority.TWO);
        SubsystemPriority.addSubsystem(m_feeder, new FeederDefault(m_feeder, m_sensors), Priority.THREE);
        SubsystemPriority.addSubsystem(m_intake, new IntakeDefault(m_intake), Priority.FOUR);
        SubsystemPriority.addSubsystem(m_climber, new ClimberDefault(m_climber, operatorControl::getLeftAxis,
                operatorControl::getRightAxis, operatorControl::getClimberOveride, operatorControl::pov), Priority.FIVE);
        SubsystemPriority.addSubsystem(m_amp, new AmpDefault(m_amp), Priority.SIX);

        // Registers the subsystems so that they run with the specified priority
        // SubsystemPriority.registerWithPriority();

        // m_auton = m_AutoMaker.fourNoteAuto(m_alliance, notePositionDetector,
        // swerveKinodynamics, m_sensors);

        // joel mar 13: the alliance command chooses which of these autos to run
        m_auton = new AllianceCommand(
                m_AutoMaker.fourNoteAuto(Alliance.Red, m_sensors),
                m_AutoMaker.fourNoteAuto(Alliance.Blue, m_sensors));

        // this illustrates how to use AutonCommand together with AllianceCommand
        Command choosableAuton = new AutonCommand(
                Map.of(
                        AutonChooser.Routine.FOUR_NOTE, new AllianceCommand(
                                m_AutoMaker.fourNoteAuto(
                                        Alliance.Red, m_sensors),
                                m_AutoMaker.fourNoteAuto(
                                        Alliance.Blue, m_sensors)),
                        AutonChooser.Routine.FIVE_NOTE, new AllianceCommand(
                                new PrintCommand("five note red goes here"),
                                new PrintCommand("five note blue goes here")),
                        AutonChooser.Routine.COMPLEMENTARY, new AllianceCommand(
                                new PrintCommand("complementary red goes here"),
                                new PrintCommand("complementary blue goes here")),
                        AutonChooser.Routine.NOTHING, new AllianceCommand(
                                new PrintCommand("nothing red goes here"),
                                new PrintCommand("nothing blue goes here"))),
                AutonChooser::routine);

        // selftest uses fields we just initialized above, so it comes last.
        m_selfTest = new SelfTestRunner(this, operatorControl::selfTestEnable);
    }

    public void beforeCommandCycle() {
        ModeSelector.selectMode(operatorControl::pov);
    }

    public void onTeleop() {
        m_shooter.reset();
        m_amp.reset();
    }

    public void onInit() {
        // m_drive.resetPose()
        m_drive.resetPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d(Math.PI)));

    }

    public void onAuto() {
        // m_drive.resetPose(new Pose2d(m_drive.getPose().getTranslation(), new
        // Rotation2d())
    }

    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

    private void onTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).onTrue(command);
    }

    public void scheduleSelfTest() {
        m_selfTest.schedule();
    }

    public void scheduleAuton() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    public void cancelAuton() {
        if (m_auton == null)
            return;
        m_auton.cancel();
    }

    // this keeps the tests from conflicting via the use of simulated HAL ports.
    public void close() {
        m_modules.close();
    }

    @Override
    public String getGlassName() {
        return "RobotContainer";
    }
}
