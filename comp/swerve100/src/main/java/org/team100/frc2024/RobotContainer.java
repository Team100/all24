package org.team100.frc2024;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.motion.IntakeNote;
import org.team100.frc2024.motion.OuttakeNote;
import org.team100.frc2024.motion.PrimitiveAuto;
import org.team100.frc2024.motion.amp.AmpSubsystem;
import org.team100.frc2024.motion.amp.PivotAmp;
import org.team100.frc2024.motion.amp.PivotToAmpPosition;
import org.team100.frc2024.motion.drivetrain.manual.ManualWithShooterLock;
import org.team100.frc2024.motion.indexer.IndexCommand;
import org.team100.frc2024.motion.indexer.IndexerSubsystem;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.intake.IntakeFactory;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.frc2024.motion.shooter.ShooterFactory;
import org.team100.lib.commands.drivetrain.CommandMaker;
import org.team100.lib.commands.drivetrain.DrawSquare;
import org.team100.lib.commands.drivetrain.DriveInACircle;
import org.team100.lib.commands.drivetrain.DriveInALittleSquare;
import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.commands.drivetrain.DriveToWaypoint3;
import org.team100.lib.commands.drivetrain.DriveWithProfile;
import org.team100.lib.commands.drivetrain.DriveWithTrajectory;
import org.team100.lib.commands.drivetrain.FancyTrajectory;
import org.team100.lib.commands.drivetrain.FullStateTrajectoryListCommand;
import org.team100.lib.commands.drivetrain.Oscillate;
import org.team100.lib.commands.drivetrain.PermissiveTrajectoryListCommand;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.Rotate;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.commands.drivetrain.Spin;
import org.team100.lib.commands.drivetrain.TrajectoryListCommand;
import org.team100.lib.commands.telemetry.MorseCodeBeep;
import org.team100.lib.config.AllianceSelector;
import org.team100.lib.config.AutonSelector;
import org.team100.lib.config.Identity;
import org.team100.lib.config.NotePoseDetector;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.controller.DrivePursuitController;
import org.team100.lib.controller.DriveRamseteController;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.copies.SwerveDrivePoseEstimator100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.localization.VisionDataProvider24;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualWithHeading;
import org.team100.lib.motion.drivetrain.manual.ManualWithNoteRotation;
import org.team100.lib.motion.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.motion.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.HeadingFactory;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Annunciator;
import org.team100.lib.telemetry.Monitor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.TrajectoryMaker;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.util.Names;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private static final double kDriveCurrentLimit = 60;
    private final Telemetry t = Telemetry.get();

    private final AutonSelector m_autonSelector;
    private final int m_autonRoutine;
    private final AllianceSelector m_allianceSelector;
    private Alliance m_alliance;
    private final NotePoseDetector m_noteDetector;

    final HeadingInterface m_heading;
    private final LEDIndicator m_indicator;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    final SwerveDriveSubsystem m_drive;
    private final SwerveModuleCollection m_modules;

    private final Command m_auton;
    private final DrawSquare m_drawCircle;

    private final SelfTestRunner m_selfTest;
    final DriveInALittleSquare m_driveInALittleSquare;
    final MorseCodeBeep m_beep;
    final Monitor m_monitor;

    // Identity-specific fields
    final IndexerSubsystem m_indexer;
    final AmpSubsystem m_amp;
    // private final ClimberSubsystem m_climber;
    final Shooter m_shooter;
    final Intake m_intake;

    // Commands
    private final PivotAmp m_pivotAmp;

    private final String m_name;

    public RobotContainer(TimedRobot robot) throws IOException {
        m_name = Names.name(this);

        final DriverControl driverControl = new DriverControlProxy();
        final OperatorControl operatorControl = new OperatorControlProxy();
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();

        // these devices only currently exist on the comp bot
        if (Identity.instance == Identity.COMP_BOT) {
            // digital inputs 0, 1, 2, 3.
            m_autonSelector = new AutonSelector();
            m_autonRoutine = m_autonSelector.routine();
            // digital inputs 4, 5
            m_allianceSelector = new AllianceSelector();
            m_alliance = m_allianceSelector.alliance();
        } else {
            m_autonSelector = null;
            m_autonRoutine = 0;
            m_allianceSelector = null;
            m_alliance = Alliance.Blue;
        }

        m_alliance = Alliance.Red;


        if (m_alliance == Alliance.Blue) {
            m_layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout("2024-crescendo.json");
        } else {
            m_layout = AprilTagFieldLayoutWithCorrectOrientation.redLayout("2024-crescendo.json");
        }
        t.log(Level.INFO, m_name, "Routine", m_autonRoutine);
        t.log(Level.INFO, m_name, "Alliance", m_alliance);

        m_indicator = new LEDIndicator(8);

        // 20 words per minute is 60 ms.
        m_beep = new MorseCodeBeep(0.06);
        // m_beep = new Beep();
        BooleanSupplier test = () -> driverControl.annunicatorTest() || m_beep.getOutput();

        m_monitor = new Monitor(new Annunciator(6), test);
        // The monitor runs less frequently than the control loop.
        robot.addPeriodic(m_monitor::periodic, 0.2);

        m_modules = SwerveModuleCollection.get(kDriveCurrentLimit, swerveKinodynamics);

        m_heading = HeadingFactory.get(swerveKinodynamics, m_modules);

        //TODO the max value is a hack for the pose estimator to ignore gyro updates. Without it the gyro offset keeps updating in the wrong places. Find the real problem
        SwerveDrivePoseEstimator100 poseEstimator = swerveKinodynamics.newPoseEstimator(
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.1, 0.1, Double.MAX_VALUE));

        VisionDataProvider24 visionDataProvider = new VisionDataProvider24(
                m_layout,
                poseEstimator,
                poseEstimator::getSampledRotation);
        visionDataProvider.enable();
        NotePosition24ArrayListener notePositionDetector = new NotePosition24ArrayListener();

        notePositionDetector.enable();

        SwerveLocal swerveLocal = new SwerveLocal(swerveKinodynamics, m_modules);

        m_drive = new SwerveDriveSubsystem(
                m_heading,
                poseEstimator,
                swerveLocal,
                driverControl::speed);
            
        m_noteDetector = new NotePoseDetector(notePositionDetector,m_drive);

        m_intake = IntakeFactory.get();
        m_shooter = ShooterFactory.get();

        m_indexer = new IndexerSubsystem(30); // NEED CAN FOR AMP MOTOR //5
        m_amp = new AmpSubsystem(28, 39);
        m_pivotAmp = new PivotAmp(m_amp, operatorControl::ampPosition);

        // show mode locks slow speed.

        ////////////////////////////
        //
        // DRIVETRAIN COMMANDS
        //

        whileTrue(driverControl::defense, m_drive.runInit(m_drive::defense));
        whileTrue(driverControl::steer0, m_drive.runInit(m_drive::steer0));
        whileTrue(driverControl::steer90, m_drive.runInit(m_drive::steer90));

        // onTrue(driverControl::resetRotation0, new SetRotation(m_drive, GeometryUtil.kRotationZero));
        onTrue(driverControl::resetRotation0, new ResetPose(m_drive, 0, 0, 0));

        onTrue(driverControl::resetRotation180, new SetRotation(m_drive, Rotation2d.fromDegrees(180)));

        onTrue(driverControl::resetPose, new ResetPose(m_drive, 0, 0, 0));

        HolonomicDriveController3 controller = new HolonomicDriveController3();

        HolonomicDriveController100 dthetaController = new HolonomicDriveController100();

        whileTrue(driverControl::rotate0, new Rotate(m_drive, m_heading, swerveKinodynamics, 0));

        m_drawCircle = new DrawSquare(m_drive, swerveKinodynamics, controller);
        whileTrue(driverControl::circle, m_drawCircle);

        TrajectoryPlanner planner = new TrajectoryPlanner(swerveKinodynamics);

        whileTrue(driverControl::driveWithFancyTrajec,
                new FancyTrajectory(m_drive, planner, swerveKinodynamics));

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
        
        // whileTrue(driverControl::test, new TrajectoryListCommand(m_drive, controller, null));

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
        DriveMotionController drivePID = new DrivePIDFController(false, 2.4, 2.4);
        whileTrue(driverControl::never,
                new DriveToWaypoint100(goal, m_drive, planner, drivePID, swerveKinodynamics));

        // Drive With Profile
        whileTrue(driverControl::driveToNote,
                new DriveWithProfile(m_noteDetector::fieldRelativePose2d, m_drive, dthetaController,
                        swerveKinodynamics));

        // 254 FF follower
        DriveMotionController driveFF = new DrivePIDFController(true, 2.4, 2.4);
        whileTrue(driverControl::never,
                new DriveToWaypoint100(goal, m_drive, planner, driveFF, swerveKinodynamics));

        // 254 Pursuit follower
        DriveMotionController drivePP = new DrivePursuitController(swerveKinodynamics);
        // whileTrue(driverControl::test,
        //         new DriveToWaypoint100(goal, m_drive, planner, drivePP, swerveKinodynamics));

        // whileTrue(driverControl::test, new Amp(m_drive::getPose, m_drive, planner, drivePID, swerveKinodynamics));

        // whileTrue(driverControl::test, new DriveWithTrajectory(m_drive, planner, drivePP, swerveKinodynamics, "src/main/deploy/choreo/crossField.traj"));

        SwerveState testState = new SwerveState(
            new State100(1, -3),
            new State100(1, -1), null
        );
        // RunCommand run = new RunCommand(() -> ShooterUtil.getAngleWhileMoving(10, 7, testState), m_shooter);

        // whileTrue(driverControl::test, run);

        // 254 Ramsete follower
        // this one seems to have a pretty high tolerance?
        DriveMotionController driveRam = new DriveRamseteController();
        whileTrue(driverControl::never,
                new DriveToWaypoint100(goal, m_drive, planner, driveRam, swerveKinodynamics));

        // little square
        m_driveInALittleSquare = new DriveInALittleSquare(m_drive);
        whileTrue(driverControl::never, m_driveInALittleSquare);

        ///////////////// OPERATOR V2//////////////////////////

        // TODO: run the intake if the camera sees a note.

        m_intake.setDefaultCommand(m_intake.run(m_intake::stop));
        // operatorControl.intake().whileTrue(m_intake.run(m_intake::intake));

        // operatorControl.outtake().whileTrue(m_intake.run(m_intake::outtake));

        whileTrue(operatorControl::intake, new IntakeNote(m_intake, m_indexer));

        whileTrue(operatorControl::outtake, new OuttakeNote(m_intake, m_indexer));

        whileTrue(operatorControl::pivotToAmpPosition, new PivotToAmpPosition(m_amp));

        // TODO: spin up the shooter whenever the robot is in range.

        m_shooter.setDefaultCommand(m_shooter.run(m_shooter::stop));
        whileTrue(operatorControl::shooter, m_shooter.run(m_shooter::forward));

        /*
         * 
         * this is another way to do speed control
         * 
         * final double kMaxShooterVelocity = 30.0;
         * m_shooter.setDefaultCommand(
         * m_shooter.run(
         * () -> m_shooter.setVelocity(
         * kMaxShooterVelocity * operatorControl.shooterSpeed())));
         * 
         * the midi control can do it too
         * 
         * m_shooter.setDefaultCommand(
         * m_shooter.run(
         * () -> m_shooter.setVelocity(
         * kMaxShooterVelocity * thirdControl.shooterSpeed())));
         */

        // TODO: intake whenever intake is running.
        // TODO: stop when note is accpeted using optical detector.
        // TODO: shoot only when the shooter is ready.

        m_indexer.setDefaultCommand(m_indexer.run(m_indexer::stop));
        whileTrue(operatorControl::index, m_indexer.run(m_indexer::index));
        whileTrue(operatorControl::index, new IndexCommand(m_indexer, () -> true));
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

        PIDController thetaController = new PIDController(1.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController omegaController = new PIDController(0.5, 0, 0);

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
                        m_noteDetector::FieldRelativeTranslation2d,
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
                new ManualWithTargetLock(
                        m_name,
                        swerveKinodynamics,
                        m_heading,
                        m_noteDetector::FieldRelativeTranslation2d,
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


        ManualWithShooterLock shooterLock = new ManualWithShooterLock(
                        m_name,
                        swerveKinodynamics,
                        m_heading,
                        thetaController,
                        omegaController,
                        0.25);

        whileTrue(driverControl::test, new PrimitiveAuto(m_drive, shooterLock, planner, drivePID, drivePP, swerveKinodynamics, m_heading));

        m_drive.setDefaultCommand(driveManually);

        m_intake.setDefaultCommand(m_intake.run(m_intake::stop));
        m_shooter.setDefaultCommand(m_shooter.run(m_shooter::stop));
        m_indexer.setDefaultCommand(m_indexer.run(m_indexer::stop));
        m_amp.setDefaultCommand(m_pivotAmp);

        m_auton = m_drive.runInit(m_drive::defense);
        // selftest uses fields we just initialized above, so it comes last.
        m_selfTest = new SelfTestRunner(this, operatorControl::selfTestEnable);
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
        m_indicator.set(State.ORANGE);
    }

    public void ledStop() {
        m_indicator.close();
    }

    public void red() {
        m_indicator.set(State.RED);
    }

    public void green() {
        m_indicator.set(State.GREEN);
    }

    // this keeps the tests from conflicting via the use of simulated HAL ports.
    public void close() {
        if (m_autonSelector != null)
            m_autonSelector.close();
        if (m_allianceSelector != null)
            m_allianceSelector.close();
        m_indicator.close();
        m_modules.close();
    }

}
