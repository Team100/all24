package org.team100.frc2024;

import java.io.IOException;
import java.util.Map;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.commands.AutonCommand;
import org.team100.frc2024.commands.Feed;
import org.team100.frc2024.commands.Lob;
import org.team100.frc2024.commands.drivetrain.manual.ManualWithAmpLock;
import org.team100.frc2024.commands.drivetrain.manual.ManualWithShooterLock;
import org.team100.frc2024.config.AutonChooser;
import org.team100.frc2024.motion.AutoMaker;
import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.ShootSmartWithRotation;
import org.team100.frc2024.motion.drivetrain.manual.AmpLockCommand;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.frc2024.motion.shooter.Ramp;
import org.team100.frc2024.motion.shooter.TestShoot;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.commands.AllianceCommand;
import org.team100.lib.commands.drivetrain.DriveToPoseSimple;
import org.team100.lib.commands.drivetrain.DriveWithProfileRotation;
import org.team100.lib.commands.drivetrain.FancyTrajectory;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.commands.drivetrain.for_testing.OscillateProfile;
import org.team100.lib.commands.drivetrain.manual.DriveManually;
import org.team100.lib.commands.drivetrain.manual.FieldManualWithNoteRotation;
import org.team100.lib.commands.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualWithFullStateHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithNoteRotation;
import org.team100.lib.commands.drivetrain.manual.ManualWithProfiledHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.commands.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.drivetrain.FullStateDriveController;
import org.team100.lib.controller.drivetrain.HolonomicDriveControllerFactory;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.controller.drivetrain.MinTimeDriveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.DrivePIDFFollower;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.follower.DriveTrajectoryFollowerFactory;
import org.team100.lib.follower.DriveTrajectoryFollowerUtil;
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
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.GyroFactory;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    public RobotContainer(TimedRobot100 robot) throws IOException {
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.COMP);
        Util.printf("Using log level %s\n", poller.getLevel().name());
        Util.println("Do not use TRACE in comp, with NT logging, it will overrun");

        final LoggerFactory fieldLogger = logging.fieldLogger;
        final FieldLogger.Log fieldLog = new FieldLogger.Log(fieldLogger);

        final LoggerFactory logger = logging.rootLogger;

        final TrajectoryVisualization viz = new TrajectoryVisualization(fieldLogger);
        final DriverControl driverControl = new DriverControlProxy(logger, async);
        final OperatorControl operatorControl = new OperatorControlProxy(async);
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();

        final SensorInterface m_sensors = switch (Identity.instance) {
            case COMP_BOT -> new CompSensors(logger, 2, 1, 4);
            default -> new MockSensors();
        };

        /////////////////////////////////
        //
        // DRIVETRAIN
        //

        final LoggerFactory driveLog = logger.child("Drive");

        m_modules = SwerveModuleCollection.get(
                driveLog,
                kDriveCurrentLimit,
                kDriveStatorLimit,
                swerveKinodynamics);
        final Gyro gyro = GyroFactory.get(
                driveLog,
                swerveKinodynamics,
                m_modules);

        // ignores the rotation derived from vision.
        final SwerveDrivePoseEstimator100 poseEstimator = swerveKinodynamics.newPoseEstimator(
                driveLog,
                gyro,
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                Timer.getFPGATimestamp());

        final AprilTagFieldLayoutWithCorrectOrientation m_layout = new AprilTagFieldLayoutWithCorrectOrientation();
        final VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                driveLog,
                m_layout,
                poseEstimator);

        final AsymSwerveSetpointGenerator setpointGenerator = new AsymSwerveSetpointGenerator(
                driveLog,
                swerveKinodynamics,
                RobotController::getBatteryVoltage);
        final SwerveLocal swerveLocal = new SwerveLocal(
                driveLog,
                swerveKinodynamics,
                setpointGenerator,
                m_modules);

        m_drive = new SwerveDriveSubsystem(
                fieldLogger,
                driveLog,
                gyro,
                poseEstimator,
                swerveLocal,
                visionDataProvider);

        final NotePosition24ArrayListener noteListener = new NotePosition24ArrayListener(
                poseEstimator);

        //////////////////////////////
        //
        // SUBSYSTEMS

        final LoggerFactory sysLog = logger.child("Subsystems");

        final FeederSubsystem feeder = new FeederSubsystem(sysLog, m_sensors);

        final Intake intake = new Intake(sysLog, m_sensors);

        m_shooter = new DrumShooter(sysLog, 3, 13, 27, 58, 100);

        // final ClimberSubsystem climber = new ClimberSubsystem(sysLog, 60, 61);

        // final AmpFeeder m_ampFeeder = new AmpFeeder(sysLog);
        // final AmpPivot m_ampPivot = new AmpPivot(sysLog);

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

        // m_ampFeeder = new AmpFeeder(sysLog);
        // m_ampPivot = new AmpPivot(sysLog);

        // final ClimberSubsystem climber = new ClimberSubsystem(sysLog, 60, 61);

        ////////////////////////////
        //
        // DRIVETRAIN COMMANDS
        //

        // NOTE: you should generally try to avoid logging in commands, it's
        // easier for subsystems to do it
        final LoggerFactory comLog = logger.child("Commands");

        // RESET ZERO
        // on xbox this is "back"
        // onTrue(driverControl::resetRotation0, new SetRotation(m_drive,
        // GeometryUtil.kRotationZero));
        onTrue(driverControl::resetRotation0, new ResetPose(m_drive, 0, 0, 0));

        // RESET 180
        // on xbox this is "start"
        onTrue(driverControl::resetRotation180, new SetRotation(m_drive, GeometryUtil.kRotation180));

        // final FullStateDriveController fullStateController = new
        // FullStateDriveController();

        // cartesian position, rotation full-state.
        HolonomicFieldRelativeController.Log hlog = new HolonomicFieldRelativeController.Log(comLog);
        final HolonomicFieldRelativeController holonomicController = HolonomicDriveControllerFactory.get(hlog);

        final DriveTrajectoryFollowerUtil util = new DriveTrajectoryFollowerUtil(comLog);
        final DriveTrajectoryFollowerFactory driveControllerFactory = new DriveTrajectoryFollowerFactory(util);
        DrivePIDFFollower.Log PIDFlog = new DrivePIDFFollower.Log(comLog);

        whileTrue(driverControl::driveWithFancyTrajec,
                new FancyTrajectory(
                        comLog,
                        m_drive,
                        driveControllerFactory.fancyPIDF(PIDFlog),
                        swerveKinodynamics));

        final DriveTrajectoryFollower drivePID = driveControllerFactory.goodPIDF(PIDFlog);

        whileTrue(driverControl::driveToNote,
                new ParallelDeadlineGroup(new DriveWithProfileRotation(
                        fieldLog,
                        noteListener::getClosestTranslation2d,
                        m_drive,
                        holonomicController,
                        swerveKinodynamics), intake.run(intake::intakeSmart)));
        // try the new mintime controller
        final HolonomicFieldRelativeController minTimeController = new MinTimeDriveController(comLog, hlog);
        whileTrue(driverControl::actualCircle,
                new DriveToPoseSimple(
                        comLog,
                        new Pose2d(8, 4, GeometryUtil.kRotationZero),
                        minTimeController,
                        m_drive));
        // whileTrue(driverControl::actualCircle,
        // new DriveInACircle(comLog, m_drive, controller, -1, viz));

        ///////////////////////
        //
        // for testing odometry
        //
        // TrajectoryMaker tmaker = new TrajectoryMaker(List.of(new
        /////////////////////// ConstantConstraint(1.0, 1.0)));
        // StraightLineTrajectory maker = new StraightLineTrajectory(false, tmaker);
        // slow, will not work for high-speed entry
        // HolonomicProfile hp = new HolonomicProfile(TimedRobot100.LOOP_PERIOD_S, 1, 1,
        // 0.01, 1, 1, 0.01);
        // high cruise but also moderate accel
        HolonomicProfile hp = new HolonomicProfile(TimedRobot100.LOOP_PERIOD_S, 1, 1, 0.01, 3, 3, 0.01);
        FullStateDriveController hcontroller = new FullStateDriveController(hlog);

        whileTrue(driverControl::fullCycle,
                // new RepeatCommand(
                // new SequentialCommandGroup(
                // new OscillateForceField(m_drive, halfFullStateController, 1),
                // new OscillateForceField(m_drive, halfFullStateController, -1))));

                new RepeatCommand(
                        new SequentialCommandGroup(
                                new OscillateProfile(m_drive, hp, hcontroller, 1),
                                new OscillateProfile(m_drive, hp, hcontroller, -1))));

        // new RepeatCommand(
        // new SequentialCommandGroup(
        // new OscillatePosition(driveLog, m_drive, maker, controller, 1, viz),
        // new OscillatePosition(driveLog, m_drive, maker, controller, -1, viz))));

        // whileTrue(driverControl::fullCycle, new OscillateDirect(comLog, m_drive));
        // new Oscillate(comLog, m_drive));
        // new RepeatCommand(
        // new FullCycle(comLog, m_drive, controller, viz)));

        // whileTrue(operatorControl::intake,
        //         new RunIntakeAndAmpFeeder(intake, feeder, m_ampFeeder));

        // whileTrue(operatorControl::outtake,
        //         new OuttakeCommand(intake, m_shooter, m_ampFeeder, feeder));

        whileTrue(operatorControl::ramp, new Ramp(m_shooter, m_drive));

        whileTrue(operatorControl::feed, new Feed(intake, feeder));

        // fast, then slow.
        // whileTrue(operatorControl::pivotToAmpPosition,
        //         new AmpFastThenSlow(m_ampPivot, 1.7, 1.8));

        // whileTrue(operatorControl::feedToAmp,
        //         new FeedToAmp(intake, m_shooter, m_ampFeeder, feeder));

        whileTrue(operatorControl::testShoot, new TestShoot(m_shooter));

        // whileTrue(operatorControl::outtakeFromAmp, m_ampFeeder.run(m_ampFeeder::outtake));

        whileTrue(operatorControl::never, new Lob(m_shooter, intake));

        // whileTrue(operatorControl::homeClimber, new HomeClimber(comLog, climber));

        // whileTrue(operatorControl::climbUpPosition, climber.upPosition());
        // whileTrue(operatorControl::climbDownPosition, climber.downPosition());

        ///////////////////////////
        //
        // DRIVE
        //

        final PIDController thetaController = new PIDController(3.0, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        final PIDController omegaController = new PIDController(0.2, 0, 0);

        final DriveManually driveManually = new DriveManually(driverControl::velocity, m_drive);
        final LoggerFactory manLog = comLog.child(driveManually);
        driveManually.register("MODULE_STATE", false,
                new SimpleManualModuleStates(manLog, swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(manLog, swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_FACING_NOTE", false,
                new ManualWithNoteRotation(
                        fieldLog,
                        manLog,
                        swerveKinodynamics,
                        noteListener::getClosestTranslation2d,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(manLog, swerveKinodynamics));

        driveManually.register("SNAPS_PROFILED", true,
                new ManualWithProfiledHeading(
                        manLog,
                        swerveKinodynamics,
                        driverControl::desiredRotation,
                        thetaController,
                        omegaController));

        // these gains are not terrible; trying to go faster seems to induce oscillation
        // theta feedback is rad/s, per rad of error.
        // so if you want to go 1.5rad at a time in a lot less than
        // a second, you kinda want 3 or 5 or something. if you make
        // it too big, it will overshoot.
        // omega feedback is rad/s per rad/s of error.
        // this generally opposes the theta feedback, so you don't
        // want it to be too large, or it will slow the motion too much
        // but you don't want it to be too small either, it helps
        // prevent overshoot.
        driveManually.register("SNAPS_FULL_STATE", true,
                new ManualWithFullStateHeading(
                        manLog,
                        swerveKinodynamics,
                        driverControl::desiredRotation,
                        new double[] {
                                5,
                                0.35
                        }));

        driveManually.register("FIELD_RELATIVE_FACING_NOTE", false,
                new FieldManualWithNoteRotation(
                        fieldLog,
                        manLog,
                        swerveKinodynamics,
                        noteListener::getClosestTranslation2d,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("LOCKED", false,
                new ManualWithTargetLock(
                        fieldLog,
                        manLog,
                        swerveKinodynamics,
                        driverControl::target,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("SHOOTER_LOCK", false,
                new ManualWithShooterLock(
                        fieldLog,
                        manLog,
                        swerveKinodynamics,
                        thetaController,
                        omegaController));

        final PIDController omega2Controller = new PIDController(0.5, 0, 0); // .5

        final ManualWithShooterLock shooterLock = new ManualWithShooterLock(
                fieldLog,
                comLog.child("ShooterLock"),
                swerveKinodynamics,
                thetaController,
                omega2Controller);

        final ManualWithAmpLock ampLock = new ManualWithAmpLock(
                fieldLog,
                comLog,
                swerveKinodynamics,
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
                swerveKinodynamics,
                viz);

        // whileTrue(driverControl::test, m_AutoMaker.citrus(Alliance.Blue));
        whileTrue(driverControl::test, m_AutoMaker.fourNoteAuto(Alliance.Blue, m_sensors));

        whileTrue(driverControl::ampLock,
                new AmpLockCommand(ampLock, driverControl::velocity, m_drive));

        whileTrue(driverControl::shooterLock,
                new ShootSmartWithRotation(comLog, m_drive, m_shooter, feeder, intake, shooterLock,
                        driverControl::velocity));

        //////////////////
        //
        // DEFAULT COMMANDS
        //

        m_drive.setDefaultCommand(driveManually);
        m_shooter.setDefaultCommand(m_shooter.run(m_shooter::stop));
        feeder.setDefaultCommand(feeder.run(feeder::stop));
        intake.setDefaultCommand(intake.run(intake::stop));
        // climber.setDefaultCommand(new ClimberDefault(
        // if far from the goal, go fast. if near, go slow.
        // m_ampPivot.setDefaultCommand(new AmpFastThenSlow(m_ampPivot, 0.1, 0));

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

}
