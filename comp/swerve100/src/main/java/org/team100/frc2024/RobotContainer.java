package org.team100.frc2024;

import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.commands.AutonCommand;
import org.team100.frc2024.commands.Feed;
import org.team100.frc2024.commands.Lob;
import org.team100.frc2024.commands.climber.HomeClimber;
import org.team100.frc2024.commands.drivetrain.DriveWithProfileNote;
import org.team100.frc2024.commands.drivetrain.manual.ManualWithAmpLock;
import org.team100.frc2024.commands.drivetrain.manual.ManualWithShooterLock;
import org.team100.frc2024.config.AutonChooser;
import org.team100.frc2024.motion.AutoMaker;
import org.team100.frc2024.motion.FeedToAmp;
import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.OuttakeCommand;
import org.team100.frc2024.motion.ShootSmartWithRotation;
import org.team100.frc2024.motion.amp.AmpFastThenSlow;
import org.team100.frc2024.motion.amp.AmpFeeder;
import org.team100.frc2024.motion.amp.AmpPivot;
import org.team100.frc2024.motion.amp.DriveToAmp;
import org.team100.frc2024.motion.climber.ClimberDefault;
import org.team100.frc2024.motion.climber.ClimberSubsystem;
import org.team100.frc2024.motion.drivetrain.manual.AmpLockCommand;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.intake.RunIntakeAndAmpFeeder;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.frc2024.motion.shooter.Ramp;
import org.team100.frc2024.motion.shooter.TestShoot;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.commands.AllianceCommand;
import org.team100.lib.commands.FullCycle;
import org.team100.lib.commands.drivetrain.DriveInACircle;
import org.team100.lib.commands.drivetrain.FancyTrajectory;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.commands.drivetrain.manual.DriveManually;
import org.team100.lib.commands.drivetrain.manual.FieldManualWithNoteRotation;
import org.team100.lib.commands.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualWithFullStateHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithMinTimeHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithNoteRotation;
import org.team100.lib.commands.drivetrain.manual.ManualWithProfiledHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.commands.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DriveMotionControllerFactory;
import org.team100.lib.controller.DriveMotionControllerUtil;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.localization.VisionDataProvider24;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.GyroFactory;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.telemetry.TelemetryLevelPoller;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Try to keep this container clean; if there's something you want to keep but
 * don't need right now, cut and paste it into {@link RobotContainerParkingLot}.
 */
public class RobotContainer implements Glassy {
    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    private static final double kDriveCurrentLimit = 50;
    private static final double kDriveStatorLimit = 100;

    private final SwerveModuleCollection m_modules;
    private final Command m_auton;
    private final DrumShooter m_shooter;
    final SwerveDriveSubsystem m_drive;
    final AmpFeeder m_ampFeeder;
    final AmpPivot m_ampPivot;

    public RobotContainer(TimedRobot100 robot) throws IOException {
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        final Telemetry telemetry = Telemetry.instance();
        new TelemetryLevelPoller(async, telemetry::setLevel, Level.TRACE);
        final SupplierLogger2 fieldLogger = telemetry.fieldLogger;
        final SupplierLogger2 logger = telemetry.rootLogger;

        final TrajectoryVisualization viz = new TrajectoryVisualization(fieldLogger);
        final DriverControl driverControl = new DriverControlProxy(logger, async);
        final OperatorControl operatorControl = new OperatorControlProxy(async);
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get(logger);

        final SensorInterface m_sensors = switch (Identity.instance) {
            case COMP_BOT -> new CompSensors(logger, 2, 1, 4);
            default -> new MockSensors();
        };

        m_modules = SwerveModuleCollection.get(
                logger,
                kDriveCurrentLimit,
                kDriveStatorLimit,
                swerveKinodynamics);
        final Gyro gyro = GyroFactory.get(
                logger,
                swerveKinodynamics,
                m_modules,
                asyncFactory);

        // ignores the rotation derived from vision.
        final SwerveDrivePoseEstimator100 poseEstimator = swerveKinodynamics.newPoseEstimator(
                logger,
                gyro.getYawNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                Timer.getFPGATimestamp());

        final AprilTagFieldLayoutWithCorrectOrientation m_layout = new AprilTagFieldLayoutWithCorrectOrientation();
        final VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                logger,
                m_layout,
                poseEstimator);

        final SwerveLocal swerveLocal = new SwerveLocal(logger, swerveKinodynamics, m_modules);

        m_drive = new SwerveDriveSubsystem(
                fieldLogger,
                logger,
                gyro,
                poseEstimator,
                swerveLocal,
                visionDataProvider);

        final NotePosition24ArrayListener noteListener = new NotePosition24ArrayListener(
                () -> m_drive.getState().pose());

        final FeederSubsystem feeder = new FeederSubsystem(logger, m_sensors);

        final Intake intake = new Intake(logger, m_sensors);

        m_shooter = new DrumShooter(logger, 3, 13, 27, 58, 100);

        ///////////////////////////
        //
        // LEDS
        //

        final LEDIndicator ledIndicator = new LEDIndicator(0);
        // has no default command, registers its own periodic.
        new LEDSubsystem(
                ledIndicator,
                m_sensors,
                m_shooter,
                visionDataProvider);

        m_ampFeeder = new AmpFeeder(logger);
        m_ampPivot = new AmpPivot(logger);

        final ClimberSubsystem climber = new ClimberSubsystem(logger, 60, 61);

        ////////////////////////////
        //
        // DRIVETRAIN COMMANDS
        //

        // RESET ZERO
        // on xbox this is "back"
        onTrue(driverControl::resetRotation0, new SetRotation(m_drive, GeometryUtil.kRotationZero));

        // RESET 180
        // on xbox this is "start"
        onTrue(driverControl::resetRotation180, new SetRotation(m_drive, GeometryUtil.kRotation180));

        // final FullStateDriveController fullStateController = new
        // FullStateDriveController();
        final HolonomicDriveController100 dthetaController = new HolonomicDriveController100(logger);

        final List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();

        final DriveMotionControllerUtil util = new DriveMotionControllerUtil(logger);
        final DriveMotionControllerFactory driveControllerFactory = new DriveMotionControllerFactory(util);
        DrivePIDFController.Log PIDFlog = new DrivePIDFController.Log(logger);

        whileTrue(driverControl::driveWithFancyTrajec,
                new FancyTrajectory(
                        logger,
                        m_drive,
                        driveControllerFactory.fancyPIDF(PIDFlog),
                        constraints));

        // 254 PID follower
        final HolonomicDriveController3 controller = new HolonomicDriveController3(logger);
        final DriveMotionController drivePID = driveControllerFactory.goodPIDF(PIDFlog);

        whileTrue(driverControl::driveToNote,
                new DriveWithProfileNote(
                        fieldLogger,
                        logger,
                        intake,
                        noteListener::getClosestTranslation2d,
                        m_drive,
                        dthetaController,
                        swerveKinodynamics));
        whileTrue(driverControl::actualCircle,
                new DriveInACircle(logger, m_drive, controller, -1, viz));

        whileTrue(driverControl::driveToAmp,
                new DriveToAmp(
                        logger,
                        m_drive,
                        swerveKinodynamics,
                        m_ampPivot,
                        m_ampFeeder,
                        intake,
                        m_shooter,
                        feeder));

        whileTrue(driverControl::fullCycle,
                new RepeatCommand(
                        new FullCycle(logger, m_drive, controller, viz)));

        whileTrue(operatorControl::intake,
                new RunIntakeAndAmpFeeder(intake, feeder, m_ampFeeder));

        whileTrue(operatorControl::outtake,
                new OuttakeCommand(intake, m_shooter, m_ampFeeder, feeder));

        whileTrue(operatorControl::ramp, new Ramp(m_shooter, m_drive));

        whileTrue(operatorControl::feed, new Feed(intake, feeder));

        // hold the amp up while holding the button
        // whileTrue(operatorControl::pivotToAmpPosition, new AmpSet(ampLogger,
        // m_ampPivot, 1.8));

        // fast, then slow.
        // TODO: tune these numbers
        whileTrue(operatorControl::pivotToAmpPosition,
                new AmpFastThenSlow(logger, m_ampPivot, 1.7, 1.8));

        whileTrue(operatorControl::feedToAmp,
                new FeedToAmp(intake, m_shooter, m_ampFeeder, feeder));

        whileTrue(operatorControl::testShoot, new TestShoot(m_shooter));

        whileTrue(operatorControl::outtakeFromAmp, m_ampFeeder.run(m_ampFeeder::outtake));

        whileTrue(operatorControl::never, new Lob(m_shooter, intake));

        whileTrue(operatorControl::homeClimber, new HomeClimber(logger, climber));

        whileTrue(operatorControl::climbUpPosition, climber.upPosition(logger));
        whileTrue(operatorControl::climbDownPosition, climber.downPosition(logger));

        ///////////////////////////
        //
        // DRIVE
        //

        // TODO (jun 24) tune theta and omega control
        // TODO replace with min-time or full-state
        final PIDController thetaController = new PIDController(2.0, 0, 0); // 1.7
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        final PIDController omegaController = new PIDController(0.1, 0, 0); // .5

        final DriveManually driveManually = new DriveManually(logger, driverControl::velocity, m_drive);

        driveManually.register("MODULE_STATE", false,
                new SimpleManualModuleStates(logger, swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(logger, swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_FACING_NOTE", false,
                new ManualWithNoteRotation(
                        fieldLogger,
                        logger,
                        swerveKinodynamics,
                        gyro,
                        noteListener::getClosestTranslation2d,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(logger, swerveKinodynamics));

        driveManually.register("SNAPS_PROFILED", true,
                new ManualWithProfiledHeading(
                        logger,
                        swerveKinodynamics,
                        gyro,
                        driverControl::desiredRotation,
                        thetaController,
                        omegaController));

        // these gains are not terrible; trying to go faster seems to induce oscillation
        driveManually.register("SNAPS_FULL_STATE", true,
                new ManualWithFullStateHeading(
                        logger,
                        swerveKinodynamics,
                        gyro,
                        driverControl::desiredRotation,
                        new double[] { 5.0, 0.5 }));

        driveManually.register("SNAPS_MIN_TIME", true,
                new ManualWithMinTimeHeading(
                        logger,
                        swerveKinodynamics,
                        gyro,
                        driverControl::desiredRotation));

        driveManually.register("FIELD_RELATIVE_FACING_NOTE", false,
                new FieldManualWithNoteRotation(
                        fieldLogger,
                        logger,
                        swerveKinodynamics,
                        gyro,
                        noteListener::getClosestTranslation2d,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("LOCKED", false,
                new ManualWithTargetLock(
                        fieldLogger,
                        logger,
                        swerveKinodynamics,
                        gyro,
                        driverControl::target,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("SHOOTER_LOCK", false,
                new ManualWithShooterLock(
                        fieldLogger,
                        logger,
                        swerveKinodynamics,
                        gyro,
                        thetaController,
                        omegaController));

        final PIDController omega2Controller = new PIDController(0.5, 0, 0); // .5

        final ManualWithShooterLock shooterLock = new ManualWithShooterLock(
                fieldLogger,
                logger,
                swerveKinodynamics,
                gyro,
                thetaController,
                omega2Controller);

        final ManualWithAmpLock ampLock = new ManualWithAmpLock(
                fieldLogger,
                logger,
                swerveKinodynamics,
                gyro,
                thetaController,
                omega2Controller);

        final AutoMaker m_AutoMaker = new AutoMaker(
                logger,
                m_drive,
                driveControllerFactory,
                drivePID,
                0,
                feeder,
                m_shooter,
                intake,
                m_sensors,
                constraints,
                viz);

        whileTrue(driverControl::test, m_AutoMaker.citrus(Alliance.Blue));

        whileTrue(driverControl::ampLock,
                new AmpLockCommand(ampLock, driverControl::velocity, m_drive));

        whileTrue(driverControl::shooterLock,
                new ShootSmartWithRotation(logger, m_drive, m_shooter, feeder, intake, shooterLock,
                        driverControl::velocity));

        //////////////////
        //
        // DEFAULT COMMANDS
        //

        m_drive.setDefaultCommand(driveManually);
        m_shooter.setDefaultCommand(m_shooter.run(m_shooter::stop));
        feeder.setDefaultCommand(feeder.run(feeder::stop));
        intake.setDefaultCommand(intake.run(intake::stop));
        climber.setDefaultCommand(new ClimberDefault(
                logger,
                climber,
                operatorControl::leftClimb,
                operatorControl::rightClimb));
        m_ampFeeder.setDefaultCommand(m_ampFeeder.run(m_ampFeeder::stop));
        // m_ampPivot.setDefaultCommand(new AmpSet(ampLogger, m_ampPivot, 0));
        // if far from the goal, go fast. if near, go slow.
        // TODO: tune these numbers
        m_ampPivot.setDefaultCommand(new AmpFastThenSlow(logger, m_ampPivot, 0.1, 0));

        ////////////////////
        //
        // AUTONOMOUS
        //

        // this illustrates how to use AutonCommand together with AllianceCommand
        m_auton = new AutonCommand(
                Map.of(
                        AutonChooser.Routine.FIVE_NOTE, new AllianceCommand(
                                m_AutoMaker.fourNoteAuto(
                                        Alliance.Red, m_sensors),
                                m_AutoMaker.fourNoteAuto(
                                        Alliance.Blue, m_sensors)),
                        AutonChooser.Routine.COMPLEMENTARY, new AllianceCommand(
                                m_AutoMaker.citrus(
                                        Alliance.Red),
                                m_AutoMaker.citrus(
                                        Alliance.Blue)),
                        AutonChooser.Routine.COMPLEMENTARY2, new AllianceCommand(
                                m_AutoMaker.citrusv2(
                                        Alliance.Red),
                                m_AutoMaker.citrusv2(
                                        Alliance.Blue)),
                        AutonChooser.Routine.SIBLING, new AllianceCommand(
                                m_AutoMaker.sibling(
                                        Alliance.Red),
                                m_AutoMaker.sibling(
                                        Alliance.Blue)),
                        AutonChooser.Routine.NOTHING, new AllianceCommand(
                                new PrintCommand("nothing red goes here"),
                                new PrintCommand("nothing blue goes here"))),
                AutonChooser::routine);
    }

    public void beforeCommandCycle() {
        // ModeSelector.selectMode(operatorControl::pov);
    }

    public void onTeleop() {
        m_shooter.reset();
    }

    public void onInit() {
        // m_drive.resetPose()
        m_drive.resetPose(new Pose2d(m_drive.getState().pose().getTranslation(), new Rotation2d(Math.PI)));

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

    public void scheduleAuton() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    public void periodic() {
        //
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
