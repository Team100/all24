package org.team100.frc2024;

import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.commands.AutonCommand;
import org.team100.frc2024.commands.Feed;
import org.team100.frc2024.commands.Lob;
import org.team100.frc2024.commands.drivetrain.DriveWithProfileNote;
import org.team100.frc2024.config.AutonChooser;
import org.team100.frc2024.motion.AutoMaker;
import org.team100.frc2024.motion.FeedToAmp;
import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.OuttakeCommand;
import org.team100.frc2024.motion.ShootSmartWithRotation;
import org.team100.frc2024.motion.amp.AmpFeeder;
import org.team100.frc2024.motion.amp.AmpPivot;
import org.team100.frc2024.motion.amp.AmpSet;
import org.team100.frc2024.motion.amp.DriveToAmp;
import org.team100.frc2024.motion.climber.ClimberDefault;
import org.team100.frc2024.motion.climber.ClimberSubsystem;
import org.team100.frc2024.motion.drivetrain.manual.AmpLockCommand;
import org.team100.frc2024.motion.drivetrain.manual.ManualWithAmpLock;
import org.team100.frc2024.motion.drivetrain.manual.ManualWithShooterLock;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.intake.RunIntakeAndAmpFeeder;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.frc2024.motion.shooter.Ramp;
import org.team100.frc2024.motion.shooter.TestShoot;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.commands.AllianceCommand;
import org.team100.lib.commands.drivetrain.DriveInACircle;
import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.FancyTrajectory;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DriveMotionControllerFactory;
import org.team100.lib.controller.FullStateDriveController;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.CameraUpdater;
import org.team100.lib.localization.FireControl;
import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.localization.VisionDataProvider24;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.manual.FieldManualWithNoteRotation;
import org.team100.lib.motion.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualWithFullStateHeading;
import org.team100.lib.motion.drivetrain.manual.ManualWithMinTimeHeading;
import org.team100.lib.motion.drivetrain.manual.ManualWithNoteRotation;
import org.team100.lib.motion.drivetrain.manual.ManualWithProfiledHeading;
import org.team100.lib.motion.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.motion.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.HeadingFactory;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.FieldLogger;
import org.team100.lib.telemetry.Logger;
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
    private final SelfTestRunner m_selfTest;
    private final DrumShooter m_shooter;
    private final CameraUpdater cameraUpdater;
    final SwerveDriveSubsystem m_drive;
    final AmpFeeder m_ampFeeder;
    final AmpPivot m_ampPivot;

    public RobotContainer(TimedRobot100 robot) throws IOException {
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        final TelemetryLevelPoller poller = new TelemetryLevelPoller(async);
        poller.setDefault(Level.TRACE);

        final Telemetry telemetry = Telemetry.get();
        final FieldLogger fieldLogger = telemetry.fieldLogger(true);
        final Logger sensorLogger = telemetry.namedRootLogger("SENSOR", false);
        final Logger driveLogger = telemetry.namedRootLogger("DRIVE", false);
        final Logger shooterLogger = telemetry.namedRootLogger("SHOOTER", false);
        final Logger intakeLogger = telemetry.namedRootLogger("INTAKE", false);
        final Logger ampLogger = telemetry.namedRootLogger("AMP", false);
        final Logger climberLogger = telemetry.namedRootLogger("CLIMBER", false);

        final TrajectoryVisualization viz = new TrajectoryVisualization(fieldLogger);
        final DriverControl driverControl = new DriverControlProxy(driveLogger, async);
        final OperatorControl operatorControl = new OperatorControlProxy(async);
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get(driveLogger);

        final SensorInterface m_sensors;
        switch (Identity.instance) {
            case COMP_BOT:
                m_sensors = new CompSensors(sensorLogger, 2, 1, 4);
                break;
            default:
                // always returns false
                m_sensors = new MockSensors();
        }

        m_modules = SwerveModuleCollection.get(
                driveLogger,
                kDriveCurrentLimit,
                kDriveStatorLimit,
                swerveKinodynamics);
        final HeadingInterface m_heading = HeadingFactory.get(
                driveLogger,
                swerveKinodynamics,
                m_modules,
                asyncFactory);

        // ignores the rotation derived from vision.
        SwerveDrivePoseEstimator100 poseEstimator = swerveKinodynamics.newPoseEstimator(
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                Timer.getFPGATimestamp());

        FireControl fireControl = new FireControl() {
        };

        final AprilTagFieldLayoutWithCorrectOrientation m_layout = new AprilTagFieldLayoutWithCorrectOrientation();
        VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                driveLogger,
                m_layout,
                poseEstimator,
                fireControl);
        visionDataProvider.enable();

        NotePosition24ArrayListener notePositionDetector = new NotePosition24ArrayListener(poseEstimator);
        notePositionDetector.enable();

        SwerveLocal swerveLocal = new SwerveLocal(driveLogger, swerveKinodynamics, m_modules);

        m_drive = new SwerveDriveSubsystem(
                fieldLogger,
                driveLogger,
                m_heading,
                poseEstimator,
                swerveLocal,
                driverControl::speed);
        cameraUpdater = new CameraUpdater(() -> poseEstimator.getEstimatedPosition().pose(), m_layout);

        final FeederSubsystem m_feeder = new FeederSubsystem(shooterLogger, m_sensors);

        final Intake m_intake = new Intake(intakeLogger, m_sensors);

        m_shooter = new DrumShooter(shooterLogger, 3, 13, 27, 58, 100);

        ///////////////////////////
        //
        // LEDS
        //

        final LEDIndicator m_indicator = new LEDIndicator(0);
        // has no default command, registers its own periodic.
        new LEDSubsystem(
                m_indicator,
                m_sensors,
                m_shooter,
                visionDataProvider);

        m_ampFeeder = new AmpFeeder(ampLogger);
        m_ampPivot = new AmpPivot(ampLogger);

        final ClimberSubsystem m_climber = new ClimberSubsystem(climberLogger, 60, 61);

        ////////////////////////////
        //
        // DRIVETRAIN COMMANDS
        //

        // RESET ZERO
        // on xbox this is "back"
        onTrue(driverControl::resetRotation0, new ResetPose(m_drive, 0, 0, 0));

        // RESET 180
        // on xbox this is "start"
        onTrue(driverControl::resetRotation180, new SetRotation(m_drive, GeometryUtil.kRotation180));
        FullStateDriveController fullStateController = new FullStateDriveController();
        HolonomicDriveController100 dthetaController = new HolonomicDriveController100(driveLogger);

        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();

        whileTrue(driverControl::driveWithFancyTrajec,
                new FancyTrajectory(driveLogger, m_drive, constraints));

        // 254 PID follower
        HolonomicDriveController3 controller = new HolonomicDriveController3(driveLogger);
        DriveMotionController drivePID = DriveMotionControllerFactory.goodPIDF(driveLogger);

        whileTrue(driverControl::driveToNote,
                new DriveWithProfileNote(
                        fieldLogger,
                        driveLogger,
                        m_intake,
                        notePositionDetector::getClosestTranslation2d,
                        m_drive,
                        dthetaController,
                        swerveKinodynamics));
        whileTrue(driverControl::actualCircle, new DriveInACircle(driveLogger, m_drive, controller, -1));

        whileTrue(driverControl::driveToAmp,
                new DriveToAmp(
                        driveLogger,
                        m_drive,
                        swerveKinodynamics,
                        m_ampPivot,
                        m_ampFeeder,
                        m_intake,
                        m_shooter,
                        m_feeder));

        whileTrue(operatorControl::intake, new RunIntakeAndAmpFeeder(m_intake, m_feeder, m_ampFeeder));

        whileTrue(operatorControl::outtake,
                new OuttakeCommand(m_intake, m_shooter, m_ampFeeder, m_feeder));

        whileTrue(operatorControl::ramp, new Ramp(m_shooter, m_drive));

        whileTrue(operatorControl::feed, new Feed(m_intake, m_feeder));

        // hold the amp up while holding the button
        whileTrue(operatorControl::pivotToAmpPosition, new AmpSet(ampLogger, m_ampPivot, 1.8));

        whileTrue(operatorControl::feedToAmp, new FeedToAmp(m_intake, m_shooter, m_ampFeeder, m_feeder));

        whileTrue(operatorControl::rezero, new TestShoot(m_shooter));

        whileTrue(operatorControl::outtakeFromAmp, m_ampFeeder.run(m_ampFeeder::outtake));

        whileTrue(operatorControl::never, new Lob(m_shooter, m_intake));

        ///////////////////////////
        //
        // DRIVE
        //

        // TODO (jun 24) tune theta and omega control
        // TODO replace with min-time or full-state
        PIDController thetaController = new PIDController(2.0, 0, 0); // 1.7
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController omegaController = new PIDController(0.1, 0, 0); // .5

        DriveManually driveManually = new DriveManually(driveLogger, driverControl::velocity, m_drive);

        driveManually.register("MODULE_STATE", false,
                new SimpleManualModuleStates(driveLogger, swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(driveLogger, swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_FACING_NOTE", false,
                new ManualWithNoteRotation(
                        fieldLogger,
                        driveLogger,
                        swerveKinodynamics,
                        m_heading,
                        notePositionDetector::getClosestTranslation2d,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(driveLogger, swerveKinodynamics));

        driveManually.register("SNAPS_PROFILED", true,
                new ManualWithProfiledHeading(
                        driveLogger,
                        swerveKinodynamics,
                        m_heading,
                        driverControl::desiredRotation,
                        thetaController,
                        omegaController));

        // these gains are not terrible; trying to go faster seems to induce oscillation
        driveManually.register("SNAPS_FULL_STATE", true,
                new ManualWithFullStateHeading(
                        driveLogger,
                        swerveKinodynamics,
                        m_heading,
                        driverControl::desiredRotation,
                        new double[] { 5.0, 0.5 }));

        driveManually.register("SNAPS_MIN_TIME", true,
                new ManualWithMinTimeHeading(
                        driveLogger,
                        swerveKinodynamics,
                        m_heading,
                        driverControl::desiredRotation));

        driveManually.register("FIELD_RELATIVE_FACING_NOTE", false,
                new FieldManualWithNoteRotation(
                        fieldLogger,
                        driveLogger,
                        swerveKinodynamics,
                        m_heading,
                        notePositionDetector::getClosestTranslation2d,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("LOCKED", false,
                new ManualWithTargetLock(
                        fieldLogger,
                        driveLogger,
                        swerveKinodynamics,
                        m_heading,
                        driverControl::target,
                        thetaController,
                        omegaController,
                        driverControl::trigger));

        driveManually.register("SHOOTER_LOCK", false,
                new ManualWithShooterLock(
                        fieldLogger,
                        driveLogger,
                        swerveKinodynamics,
                        m_heading,
                        thetaController,
                        omegaController));

        PIDController omega2Controller = new PIDController(0.5, 0, 0); // .5

        ManualWithShooterLock shooterLock = new ManualWithShooterLock(
                fieldLogger,
                driveLogger,
                swerveKinodynamics,
                m_heading,
                thetaController,
                omega2Controller);

        ManualWithAmpLock ampLock = new ManualWithAmpLock(
                fieldLogger,
                driveLogger,
                swerveKinodynamics,
                m_heading,
                thetaController,
                omega2Controller);

        AutoMaker m_AutoMaker = new AutoMaker(
                driveLogger,
                m_drive,
                drivePID,
                0,
                m_feeder,
                m_shooter,
                m_intake,
                m_sensors,
                notePositionDetector,
                constraints,
                viz);

        whileTrue(driverControl::test, m_AutoMaker.citrus(Alliance.Blue));

        whileTrue(driverControl::ampLock,
                new AmpLockCommand(ampLock, driverControl::velocity, m_drive));

        whileTrue(driverControl::shooterLock,
                new ShootSmartWithRotation(driveLogger, m_drive, m_shooter, m_feeder, m_intake, shooterLock,
                        driverControl::velocity));

        //////////////////
        //
        // DEFAULT COMMANDS
        //

        m_drive.setDefaultCommand(driveManually);
        m_shooter.setDefaultCommand(m_shooter.run(m_shooter::stop));
        m_feeder.setDefaultCommand(m_feeder.run(m_feeder::stop));
        m_intake.setDefaultCommand(m_intake.run(m_intake::stop));
        m_climber.setDefaultCommand(new ClimberDefault(
                m_climber,
                operatorControl::getLeftAxis,
                operatorControl::getRightAxis,
                operatorControl::pov));
        m_ampFeeder.setDefaultCommand(m_ampFeeder.run(m_ampFeeder::stop));
        m_ampPivot.setDefaultCommand(new AmpSet(ampLogger, m_ampPivot, 0));

        ////////////////////
        //
        // AUTONOMOUS
        //

        // this illustrates how to use AutonCommand together with AllianceCommand
        Command choosableAuton = new AutonCommand(
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
        m_auton = choosableAuton;

        // selftest uses fields we just initialized above, so it comes last.
        m_selfTest = new SelfTestRunner(this, operatorControl::selfTestEnable);
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

    public void scheduleSelfTest() {
        m_selfTest.schedule();
    }

    public void scheduleAuton() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    public void periodic() {
        if (Experiments.instance.enabled(Experiment.UseCameraUpdater))
            cameraUpdater.update();
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
