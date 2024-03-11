package org.team100.frc2024;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.RobotState100.AmpState100;
import org.team100.frc2024.RobotState100.FeederState100;
import org.team100.frc2024.RobotState100.IntakeState100;
import org.team100.frc2024.RobotState100.ShooterState100;
import org.team100.frc2024.commands.drivetrain.DriveWithProfileNote;
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
import org.team100.frc2024.motion.climber.ClimberSubsystem;
import org.team100.frc2024.motion.drivetrain.manual.ManualWithShooterLock;
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
import org.team100.lib.commands.drivetrain.CommandMaker;
import org.team100.lib.commands.drivetrain.DrawSquare;
import org.team100.lib.commands.drivetrain.DriveInACircle;
import org.team100.lib.commands.drivetrain.DriveInALittleSquare;
import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.commands.drivetrain.DriveToWaypoint3;
import org.team100.lib.commands.drivetrain.FancyTrajectory;
import org.team100.lib.commands.drivetrain.FullStateTrajectoryListCommand;
import org.team100.lib.commands.drivetrain.Oscillate;
import org.team100.lib.commands.drivetrain.PermissiveTrajectoryListCommand;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.Rotate;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.commands.drivetrain.Spin;
import org.team100.lib.commands.drivetrain.TrajectoryListCommand;
import org.team100.lib.config.AllianceSelector;
import org.team100.lib.config.AutonSelector;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.controller.DrivePursuitController;
import org.team100.lib.controller.DriveRamseteController;
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
import org.team100.lib.indicator.LEDStrip;
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
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.TrajectoryMaker;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.util.Names;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer implements Glassy {
    private static final double kDriveCurrentLimit = 60;
    private final Telemetry t = Telemetry.get();

    private final AutonSelector m_autonSelector;
    private final int m_autonRoutine;
    private final AllianceSelector m_allianceSelector;
    private Alliance m_alliance;

    final HeadingInterface m_heading;
    private final LEDIndicator m_indicator;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    final SwerveDriveSubsystem m_drive;
    private final SwerveModuleCollection m_modules;

    private final Command m_auton;
    private final DrawSquare m_drawCircle;

    private final SelfTestRunner m_selfTest;
    final DriveInALittleSquare m_driveInALittleSquare;

    // joel 2/22/24 removing for SVR, put back after that.
    // final MorseCodeBeep m_beep;
    // final Monitor m_monitor;

    // Identity-specific fields
    // final IndexerSubsystem m_indexer;
    final AmpSubsystem m_amp;
    private final ClimberSubsystem m_climber;
    final Shooter m_shooter;
    final Intake m_intake;
    final LEDSubsystem m_ledSubsystem;
    final SensorInterface m_sensors;
    final FeederSubsystem m_feeder;
    // final SwerveDriveSubsystem m_drive;

    final DriverControl driverControl;
    final OperatorControl operatorControl;
    // Commands
    private final PivotAmp m_pivotAmp;

    private final String m_name;

    public RobotContainer(TimedRobot robot) throws IOException {
        m_name = Names.name(this);

        driverControl = new DriverControlProxy();
        operatorControl = new OperatorControlProxy();
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();

        // these devices only currently exist on the comp bot
        if (Identity.instance == Identity.COMP_BOT) {
            // digital inputs 0, 1, 2, 3.
            // m_autonSelector = new AutonSelector();
            // m_autonRoutine = m_autonSelector.routine();
            // digital inputs 4, 5
            // m_allianceSelector = new AllianceSelector();
            // m_alliance = Selector.alliance();
            m_autonSelector = null;
            m_autonRoutine = 0;
            m_allianceSelector = null;
            m_alliance = Alliance.Blue;
        } else {
            m_autonSelector = null;
            m_autonRoutine = 0;
            m_allianceSelector = null;
            m_alliance = Alliance.Blue;
        }

        // *************************
        //
        // override the alliance logic.
        // switch(Identity.instance) {
        // case BLANK:
        // m_alliance = Alliance.Blue;
        // break;
        // default:
        // m_alliance = DriverStation.getAlliance().get();
        // break;
        // }

        // m_alliance = DriverStation.getAlliance().get();F

        // if(m_alliance == null){
        m_alliance = Alliance.Blue;
        // }

        // m_alliance = Alliance.Blue ;
        // m_alliance = Allsiance.Blue;

        if (m_alliance == Alliance.Blue) {
            m_layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout("2024-crescendo.json");
        } else {
            m_layout = AprilTagFieldLayoutWithCorrectOrientation.redLayout("2024-crescendo.json");
        }

        t.log(Level.INFO, m_name, "Routine", m_autonRoutine);
        t.log(Level.INFO, m_name, "Alliance", m_alliance);

        switch (Identity.instance) {
            case COMP_BOT:
                m_sensors = new CompSensors(8, 9); // Definitely real numbers
                break;
            default:
                // always returns false
                m_sensors = new MockSensors();
        }

        // joel 2/22/24 removing for SVR, put it back after that.
        // 20 words per minute is 60 ms.
        // m_beep = new MorseCodeBeep(0.06);
        // m_beep = new Beep();
        // BooleanSupplier test = () -> driverControl.annunicatorTest() ||
        // m_beep.getOutput();
        // m_monitor = new Monitor(new Annunciator(6), test);
        // The monitor runs less frequently than the control loop.
        // robot.addPeriodic(m_monitor::periodic, 0.2);

        m_modules = SwerveModuleCollection.get(kDriveCurrentLimit, swerveKinodynamics);

        m_heading = HeadingFactory.get(swerveKinodynamics, m_modules);

        // TODO the max value is a hack for the pose estimator to ignore gyro updates.
        // Without it the gyro offset keeps updating in the wrong places. Find the real
        // problem
        SwerveDrivePoseEstimator100 poseEstimator = swerveKinodynamics.newPoseEstimator(
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE)); //0.1 0.1

        VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                m_layout,
                poseEstimator,
                poseEstimator::getSampledRotation,
                m_alliance);
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

        LEDStrip strip1 = new LEDStrip(160, 0);

        m_indicator = new LEDIndicator(0, strip1);

        m_shooter = new DrumShooter(44, 45, 28, 39, 58);

        m_ledSubsystem = new LEDSubsystem(m_indicator, m_sensors, m_shooter);

        // / = new IndexerSubsystem(63); // NEED CAN FOR AMP MOTOR //5
        m_amp = new AmpSubsystem(19);
        m_pivotAmp = new PivotAmp(m_amp, operatorControl::ampPosition);

        m_climber = new ClimberSubsystem(60, 61);

        // show mode locks slow speed.

        ////////////////////////////
        //
        // DRIVETRAIN COMMANDS
        //

        whileTrue(driverControl::defense, m_drive.runInit(m_drive::defense));
        whileTrue(driverControl::steer0, m_drive.runInit(m_drive::steer0));
        whileTrue(driverControl::steer90, m_drive.runInit(m_drive::steer90));

        // onTrue(driverControl::resetRotation0, new SetRotation(m_drive,
        // GeometryUtil.kRotationZero));
        onTrue(driverControl::resetRotation0, new ResetPose(m_drive, 0, 0, 0));

        // this is @sanjan's version from some sort of vision testing in february
        // onTrue(driverControl::resetRotation0, new ResetPose(m_drive, 1.77, 1.07,
        // 2.44346));

        // on xbox this is "start"
        onTrue(driverControl::resetRotation180, new SetRotation(m_drive, Rotation2d.fromDegrees(180)));

        // on xbox this is left bumper
        // on joystick this is button 4
        // on starting zone line lined up with note
        onTrue(driverControl::resetPose, new ResetPose(m_drive, .5, 7, 0));

        HolonomicDriveController3 controller = new HolonomicDriveController3();

        HolonomicDriveController100 dthetaController = new HolonomicDriveController100();

        whileTrue(driverControl::rotate0, new Rotate(m_drive, m_heading, swerveKinodynamics, 0));

        m_drawCircle = new DrawSquare(m_drive, swerveKinodynamics, controller);
        // whileTrue(driverControl::circle, m_drawCircle);

        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();

        TrajectoryPlanner planner = new TrajectoryPlanner();

        whileTrue(driverControl::driveWithFancyTrajec,
                new FancyTrajectory(m_drive, planner, constraints));

        whileTrue(driverControl::never, new DriveInACircle(m_drive, controller, -1));
        whileTrue(driverControl::never, new Spin(m_drive, controller));
        whileTrue(driverControl::never, new Oscillate(m_drive));

        // make a one-meter line
        whileTrue(driverControl::never,
                new TrajectoryListCommand(m_drive, controller,
                        x -> List.of(TrajectoryMaker.line(swerveKinodynamics, x))));

        // make a one-meter square
        whileTrue(driverControl::never,
                new TrajectoryListCommand(m_drive, controller,
                        x -> TrajectoryMaker.square(swerveKinodynamics, x)));

        // whileTrue(driverControl::test, new TrajectoryListCommand(m_drive, controller,
        // null));

        // one-meter square with reset at the corners
        whileTrue(driverControl::never,
                new PermissiveTrajectoryListCommand(m_drive, controller,
                        TrajectoryMaker.permissiveSquare(swerveKinodynamics)));

        // one-meter square with position and velocity feedback control
        whileTrue(driverControl::never,
                new FullStateTrajectoryListCommand(m_drive,
                        x -> TrajectoryMaker.square(swerveKinodynamics, x)));

        // trying the new ChoreoLib
        ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory("test");
        whileTrue(driverControl::never, CommandMaker.choreo(choreoTrajectory, m_drive));

        // playing with trajectory followers
        TrajectoryConfig config = new TrajectoryConfig(5, 5);
        StraightLineTrajectory maker = new StraightLineTrajectory(config);

        // field center, roughly, facing to the left.
        Pose2d goal = new Pose2d(1.877866, 7.749999, GeometryUtil.kRotation90);
        Command follower = new DriveToWaypoint3(goal, m_drive, maker, controller);
        // whileTrue(driverControl::test, follower);

        // 254 PID follower
        // joel 20240311 changed ptheta from 1 to 1.3
        DriveMotionController drivePID = new DrivePIDFController(false, 1, 1.3);
        whileTrue(driverControl::never,
                new DriveToWaypoint100(
                        goal,
                        m_drive,
                        planner,
                        drivePID,
                        constraints,
                        1));

        // Drive With Profile
        whileTrue(driverControl::driveToNote,
                new DriveWithProfileNote(notePositionDetector::getClosestTranslation2d, m_drive, dthetaController,
                        swerveKinodynamics, () -> !m_sensors.getIntakeSensor(), m_intake));

        // 254 FF follower
        // joel 20240311 changed ptheta from 2.4 to 1.3
        DriveMotionController driveFF = new DrivePIDFController(true, 2.4, 1.3);
        whileTrue(driverControl::never,
                new DriveToWaypoint100(
                        goal,
                        m_drive,
                        planner,
                        driveFF,
                        constraints,
                        1));

        // 254 Pursuit follower
        DriveMotionController drivePP = new DrivePursuitController(swerveKinodynamics);
        // whileTrue(driverControl::test,
        // new DriveToWaypoint100(goal, m_drive, planner, drivePP, swerveKinodynamics));

        // whileTrue(driverControl::test,
        // new DriveToState100(goal, new Twist2d(2, 0, 0), m_drive, planner, drivePP,
        // swerveKinodynamics));

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

        // 254 Ramsete follower
        // this one seems to have a pretty high tolerance?
        DriveMotionController driveRam = new DriveRamseteController();
        whileTrue(driverControl::never,
                new DriveToWaypoint100(
                        goal,
                        m_drive,
                        planner,
                        driveRam,
                        constraints,
                        1));

        // little square
        m_driveInALittleSquare = new DriveInALittleSquare(m_drive);
        whileTrue(driverControl::never, m_driveInALittleSquare);

        whileTrue(operatorControl::intake,
                new StartEndCommand(() -> RobotState100.changeIntakeState(IntakeState100.INTAKE),
                        () -> RobotState100.changeIntakeState(IntakeState100.STOP)));

        whileTrue(operatorControl::outtake,
                new OuttakeCommand(m_intake, m_shooter, m_amp, m_feeder));

        whileTrue(operatorControl::ramp,
                new SetDefaultShoot(m_shooter, ShooterState100.DEFAULTSHOOT));

        whileTrue(operatorControl::feed, new StartEndCommand(() -> RobotState100.changeFeederState(FeederState100.FEED),
                () -> RobotState100.changeFeederState(FeederState100.STOP)));

        // whileTrue(operatorControl::pivotToAmpPosition,
        // new ChangeAmpState(AmpState100.UP, m_amp));

        whileTrue(operatorControl::pivotToAmpPosition, new ChangeAmpState(AmpState100.UP, m_amp));

        // whileTrue(operatorControl::pivotToDownPosition, new Test2());

        whileTrue(operatorControl::pivotToDownPosition, new ChangeAmpState(AmpState100.DOWN, m_amp));

        whileTrue(operatorControl::feedToAmp, new FeedCommand(m_intake, m_shooter, m_amp, m_feeder));

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
                0,
                m_alliance,
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
        whileTrue(driverControl::shooterLock,
                m_AutoMaker.fourNoteAuto(notePositionDetector, swerveKinodynamics, m_sensors));
        // whileTrue(driverControl::test, new DriveToState101(new Pose2d(15.446963,
        // 1.522998, Rotation2d.fromDegrees(-60)), new Twist2d(0, 0, 0), m_drive,
        // planner, drivePID, swerveKinodynamics));
        // AutoMaker m_AutoMaker = new AutoMaker(m_drive, planner, drivePID,
        // swerveKinodynamics, 0, m_alliance);
        whileTrue(driverControl::test, m_AutoMaker.fourNoteAuto(notePositionDetector, swerveKinodynamics, m_sensors));
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
                operatorControl::getRightAxis, operatorControl::getClimberOveride), Priority.FIVE);
        SubsystemPriority.addSubsystem(m_amp, new AmpDefault(m_amp), Priority.SIX);

        // Registers the subsystems so that they run with the specified priority
        // SubsystemPriority.registerWithPriority();

        m_auton = m_AutoMaker.fourNoteAuto(notePositionDetector, swerveKinodynamics, m_sensors);

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

    public double getRoutine() {
        return m_autonSelector.routine();
    }

    public void ledStart() {
        // m_indicator.set(State.ORANGE);
    }

    public void ledStop() {
        // m_indicator.close();
    }

    public void red() {
        // m_indicator.set(State.RED);
    }

    public void green() {
        // m_indicator.set(State.GREEN);
    }

    // this keeps the tests from conflicting via the use of simulated HAL ports.
    public void close() {
        if (m_autonSelector != null)
            m_autonSelector.close();
        if (m_allianceSelector != null)
            m_allianceSelector.close();
        // m_indicator.close();
        m_modules.close();
    }

    @Override
    public String getGlassName() {
        return "RobotContainer";
    }

}
