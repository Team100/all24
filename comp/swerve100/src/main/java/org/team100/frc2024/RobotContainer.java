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
import org.team100.frc2024.motion.amp.AmpFeeder;
import org.team100.frc2024.motion.amp.AmpPivot;
import org.team100.frc2024.motion.amp.AmpSet;
import org.team100.frc2024.motion.climber.ClimberDefault;
import org.team100.frc2024.motion.climber.ClimberSubsystem;
import org.team100.frc2024.motion.drivetrain.manual.AmpLockCommand;
import org.team100.frc2024.motion.drivetrain.manual.ManualWithAmpLock;
import org.team100.frc2024.motion.drivetrain.manual.ManualWithShooterLock;
import org.team100.frc2024.motion.drivetrain.manual.ShooterLockCommand;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.intake.RunIntakeAndAmpFeeder;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.frc2024.motion.shooter.Ramp;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.frc2024.motion.shooter.TestShoot;
import org.team100.lib.commands.AllianceCommand;
import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.FancyTrajectory;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DriveMotionControllerFactory;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
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
import org.team100.lib.motion.drivetrain.manual.ManualWithHeading;
import org.team100.lib.motion.drivetrain.manual.ManualWithNoteRotation;
import org.team100.lib.motion.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.motion.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.HeadingFactory;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.util.Names;

import edu.wpi.first.math.VecBuilder;
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
    private static final double kDriveCurrentLimit = 50;

    private final SwerveModuleCollection m_modules;
    private final Command m_auton;
    private final SelfTestRunner m_selfTest;
    private final Shooter m_shooter;
    private final String m_name;
    private final CameraUpdater cameraUpdater;
    final SwerveDriveSubsystem m_drive;
    final AmpFeeder m_ampFeeder;
    final AmpPivot m_ampPivot;

    public RobotContainer() throws IOException {
        m_name = Names.name(this);

        final DriverControl driverControl = new DriverControlProxy();
        final OperatorControl operatorControl = new OperatorControlProxy();
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();

        final SensorInterface m_sensors;
        switch (Identity.instance) {
            case COMP_BOT:
                m_sensors = new CompSensors(2, 1, 4);
                break;
            default:
                // always returns false
                m_sensors = new MockSensors();
        }

        m_modules = SwerveModuleCollection.get(kDriveCurrentLimit, swerveKinodynamics);

        final HeadingInterface m_heading = HeadingFactory.get(swerveKinodynamics, m_modules);

        // these are the old numbers, just used as defaults. see VisionDataProvider24
        // for updated stddevs.
        double stateStdDev = 0.1;
        double visionStdDev = 0.5;

        // ignores the rotation derived from vision.
        SwerveDrivePoseEstimator100 poseEstimator = swerveKinodynamics.newPoseEstimator(
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                Timer.getFPGATimestamp(),
                VecBuilder.fill(stateStdDev, stateStdDev, 0.1),
                VecBuilder.fill(visionStdDev, visionStdDev, Double.MAX_VALUE)); // 0.1 0.1

        FireControl fireControl = new FireControl() {
        };

        final AprilTagFieldLayoutWithCorrectOrientation m_layout = new AprilTagFieldLayoutWithCorrectOrientation();
        VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                m_layout,
                poseEstimator,
                fireControl);
        visionDataProvider.enable();

        NotePosition24ArrayListener notePositionDetector = new NotePosition24ArrayListener(poseEstimator);
        notePositionDetector.enable();

        SwerveLocal swerveLocal = new SwerveLocal(swerveKinodynamics, m_modules);

        m_drive = new SwerveDriveSubsystem(
                m_heading,
                poseEstimator,
                swerveLocal,
                driverControl::speed);
        cameraUpdater = new CameraUpdater(() -> poseEstimator.getEstimatedPosition().pose(), m_layout);

        final FeederSubsystem m_feeder = new FeederSubsystem(m_sensors);

        final Intake m_intake = new Intake(m_sensors);

        m_shooter = new DrumShooter(3, 13, 27, 58);

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

        m_ampFeeder = new AmpFeeder();
        m_ampPivot = new AmpPivot();

        final ClimberSubsystem m_climber = new ClimberSubsystem(60, 61);

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

        HolonomicDriveController100 dthetaController = new HolonomicDriveController100();

        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();

        whileTrue(driverControl::driveWithFancyTrajec,
                new FancyTrajectory(m_drive, constraints));

        // 254 PID follower
        DriveMotionController drivePID = DriveMotionControllerFactory.goodPIDF();

        // Drive With Profile
        whileTrue(driverControl::driveToNote,
                new DriveWithProfileNote(
                        m_intake,
                        notePositionDetector::getClosestTranslation2d,
                        m_drive,
                        dthetaController,
                        swerveKinodynamics));

        whileTrue(operatorControl::intake, new RunIntakeAndAmpFeeder(m_intake, m_feeder, m_ampFeeder));

        whileTrue(operatorControl::outtake,
                new OuttakeCommand(m_intake, m_shooter, m_ampFeeder, m_feeder));

        whileTrue(operatorControl::ramp, new Ramp(m_shooter, m_drive));

        whileTrue(operatorControl::feed, new Feed(m_intake, m_feeder));

        // hold the amp up while holding the button
        whileTrue(operatorControl::pivotToAmpPosition, new AmpSet(m_ampPivot, 1.8));

        whileTrue(operatorControl::feedToAmp, new FeedToAmp(m_intake, m_shooter, m_ampFeeder, m_feeder));

        whileTrue(operatorControl::rezero, new TestShoot(m_shooter));

        whileTrue(operatorControl::outtakeFromAmp, m_ampFeeder.run(m_ampFeeder::outtake));

        whileTrue(operatorControl::never, new Lob(m_shooter, m_intake));

        ///////////////////////////
        //
        // DRIVE
        //

        PIDController thetaController = new PIDController(2.5, 0, 0); // 1.7
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController omegaController = new PIDController(0, 0, 0); // .5

        DriveManually driveManually = new DriveManually(driverControl::velocity, m_drive);

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
                        omegaController));

        PIDController omega2Controller = new PIDController(0, 0, 0); // .5

        ManualWithShooterLock shooterLock = new ManualWithShooterLock(
                m_name,
                swerveKinodynamics,
                m_heading,
                thetaController,
                omega2Controller);

        ManualWithAmpLock ampLock = new ManualWithAmpLock(
                m_name,
                swerveKinodynamics,
                m_heading,
                thetaController,
                omega2Controller);

        AutoMaker m_AutoMaker = new AutoMaker(
                m_drive,
                drivePID,
                0,
                m_feeder,
                m_shooter,
                m_intake,
                m_sensors,
                notePositionDetector,
                constraints);

        whileTrue(driverControl::test, m_AutoMaker.citrus(Alliance.Blue));

        whileTrue(driverControl::ampLock,
                new AmpLockCommand(ampLock, driverControl::velocity, m_drive));

        whileTrue(driverControl::shooterLock,
                new ShooterLockCommand(shooterLock, driverControl::velocity, m_drive));

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
        m_ampPivot.setDefaultCommand(new AmpSet(m_ampPivot, 0));

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
