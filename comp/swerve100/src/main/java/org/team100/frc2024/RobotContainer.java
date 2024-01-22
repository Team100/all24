package org.team100.frc2024;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.motion.amp.AmpSubsystem;
import org.team100.frc2024.motion.amp.PivotAmp;
import org.team100.frc2024.motion.climber.ClimberSubsystem;
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
import org.team100.lib.commands.drivetrain.FancyTrajectory;
import org.team100.lib.commands.drivetrain.FullStateTrajectoryListCommand;
import org.team100.lib.commands.drivetrain.ManualMode;
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
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.controller.DrivePursuitController;
import org.team100.lib.controller.DriveRamseteController;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.ControlFactory;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.ThirdControl;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.Blip24ArrayListener;
import org.team100.lib.localization.VisionDataProvider;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.selftest.SelfTestRunner;
import org.team100.lib.selftest.SelfTestable;
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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer implements SelfTestable {
    private static final double kDriveCurrentLimit = 60;
    private final Telemetry t = Telemetry.get();
    private final AutonSelector m_autonSelector;
    private final AllianceSelector m_allianceSelector;
    private final HeadingInterface m_heading;
    private final LEDIndicator m_indicator;
    private final AprilTagFieldLayoutWithCorrectOrientation layout;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveModuleCollection m_modules;

    private final Command m_auton;

    private final DrawSquare m_drawCircle;
    private final SelfTestRunner m_selfTest;
    private final DriveInALittleSquare m_driveInALittleSquare;
    private final MorseCodeBeep m_beep;
    private final Monitor m_monitor;

    // Identity-specific fields
    private final IndexerSubsystem m_indexer;
    private final AmpSubsystem m_amp;
    // private final ClimberSubsystem m_climber;
    private final Shooter m_shooter;
    private final Intake m_intake;

    //Commands
    private final PivotAmp m_pivotAmp;

    private final String m_name;

    public RobotContainer(TimedRobot robot) throws IOException {
        m_name = Names.name(this);
        // selects the correct control class for whatever is plugged in
        final ControlFactory controlFactory = new ControlFactory();
        final DriverControl driverControl = controlFactory.getDriverControl();
        final OperatorControl operatorControl = controlFactory.getOperatorControl();

        // digital inputs 0, 1, 2, 3.
        m_autonSelector = new AutonSelector();
        t.log(Level.INFO, m_name, "Routine", getRoutine());

        // digital inputs 4, 5
        m_allianceSelector = new AllianceSelector();
        t.log(Level.INFO, m_name, "Alliance", m_allianceSelector.alliance().name());

        m_indicator = new LEDIndicator(8);

        // 20 words per minute is 60 ms.
        m_beep = new MorseCodeBeep(0.06);
        // m_beep = new Beep();
        BooleanSupplier test = () -> driverControl.annunicatorTest() || m_beep.getOutput();

        m_monitor = new Monitor(new Annunciator(6), test);
        // The monitor runs less frequently than the control loop.
        robot.addPeriodic(m_monitor::periodic, 0.2);

        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();

        m_modules = SwerveModuleCollection.get(kDriveCurrentLimit, swerveKinodynamics);

        m_heading = HeadingFactory.get(swerveKinodynamics, m_modules);

        SwerveDrivePoseEstimator poseEstimator = swerveKinodynamics.newPoseEstimator(
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.1, 0.1, 0.4));

        if (m_allianceSelector.alliance() == Alliance.Blue) {
            layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout("2023-studies.json");
        } else {
            layout = AprilTagFieldLayoutWithCorrectOrientation.redLayout("2023-studies.json");
        }

        VisionDataProvider visionDataProvider = new VisionDataProvider(
                layout,
                poseEstimator,
                poseEstimator::getEstimatedPosition);
        visionDataProvider.enable();

        Blip24ArrayListener listener = new Blip24ArrayListener();
        listener.enable();

        SwerveLocal swerveLocal = new SwerveLocal(swerveKinodynamics, m_modules);

        m_intake = IntakeFactory.get(SubsystemChoice.WheelIntake, 13, -1); //3 6
        m_shooter = ShooterFactory.get(SubsystemChoice.DrumShooter, 5, 4); //7 8

        m_indexer = new IndexerSubsystem(30); // NEED CAN FOR AMP MOTOR //5

        // m_climber = new ClimberSubsystem(2, 9);
        m_amp = new AmpSubsystem(28, 39);

        m_pivotAmp = new PivotAmp(m_amp, () -> operatorControl.climberState());

        // show mode locks slow speed.
        m_drive = new SwerveDriveSubsystem(
                m_heading,
                poseEstimator,
                swerveLocal,
                driverControl::speed);

        ////////////////////////////
        //
        // DRIVETRAIN COMMANDS
        //

        driverControl.defense().whileTrue(m_drive.runInit(m_drive::defense));
        driverControl.steer0().whileTrue(m_drive.runInit(m_drive::steer0));
        driverControl.steer90().whileTrue(m_drive.runInit(m_drive::steer90));

        driverControl.resetRotation0().onTrue(new SetRotation(m_drive, GeometryUtil.kRotationZero));
        driverControl.resetRotation180().onTrue(new SetRotation(m_drive, Rotation2d.fromDegrees(180)));

        ManualMode manualMode = new ManualMode();

        driverControl.resetPose().onTrue(new ResetPose(m_drive, 0, 0, Math.PI));

        HolonomicDriveController3 controller = new HolonomicDriveController3();

        driverControl.rotate0().whileTrue(new Rotate(m_drive, m_heading, swerveKinodynamics, 0));

        m_drawCircle = new DrawSquare(m_drive, swerveKinodynamics, controller);
        driverControl.circle().whileTrue(m_drawCircle);

        TrajectoryPlanner planner = new TrajectoryPlanner(swerveKinodynamics);

        driverControl.driveWithFancyTrajec().whileTrue(
                new FancyTrajectory(m_drive, planner, swerveKinodynamics));

        driverControl.never().whileTrue(new DriveInACircle(m_drive, controller, -1));
        driverControl.never().whileTrue(new Spin(m_drive, controller));
        driverControl.never().whileTrue(new Oscillate(m_drive));

        // make a one-meter line
        driverControl.never().whileTrue(
                new TrajectoryListCommand(m_drive, controller,
                        x -> List.of(TrajectoryMaker.line(swerveKinodynamics, x))));

        // make a one-meter square
        driverControl.never().whileTrue(
                new TrajectoryListCommand(m_drive, controller,
                        x -> TrajectoryMaker.square(swerveKinodynamics, x)));

        // one-meter square with reset at the corners
        driverControl.never().whileTrue(
                new PermissiveTrajectoryListCommand(m_drive, controller,
                        TrajectoryMaker.permissiveSquare(swerveKinodynamics)));

        // one-meter square with position and velocity feedback control
        driverControl.never().whileTrue(
                new FullStateTrajectoryListCommand(m_drive,
                        x -> TrajectoryMaker.square(swerveKinodynamics, x)));

        // trying the new ChoreoLib
        ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory("test");
        driverControl.never().whileTrue(CommandMaker.choreo(choreoTrajectory, m_drive));

        // playing with trajectory followers
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        StraightLineTrajectory maker = new StraightLineTrajectory(config);

        // field center, roughly, facing to the left.
        Pose2d goal = new Pose2d(8, 4, GeometryUtil.kRotation90);
        Command follower = new DriveToWaypoint3(goal, m_drive, maker, controller);
        driverControl.never().whileTrue(follower);

        // 254 PID follower
        DriveMotionController drivePID = new DrivePIDFController(false);
        driverControl.never().whileTrue(
                new DriveToWaypoint100(goal, m_drive, planner, drivePID, swerveKinodynamics));

        // 254 FF follower
        DriveMotionController driveFF = new DrivePIDFController(true);
        driverControl.never().whileTrue(
                new DriveToWaypoint100(goal, m_drive, planner, driveFF, swerveKinodynamics));

        // 254 Pursuit follower
        DriveMotionController drivePP = new DrivePursuitController(swerveKinodynamics);
        driverControl.actualCircle().whileTrue(
                new DriveToWaypoint100(goal, m_drive, planner, drivePP, swerveKinodynamics));

        // 254 Ramsete follower
        // this one seems to have a pretty high tolerance?
        DriveMotionController driveRam = new DriveRamseteController();
        driverControl.never().whileTrue(
                new DriveToWaypoint100(goal, m_drive, planner, driveRam, swerveKinodynamics));

        // little square
        m_driveInALittleSquare = new DriveInALittleSquare(m_drive);
        driverControl.never().whileTrue(m_driveInALittleSquare);

        ///////////////// OPERATOR V2//////////////////////////

        // TODO: run the intake if the camera sees a note.
        m_intake.setDefaultCommand(m_intake.run(() -> m_intake.setIntake(0)));
        operatorControl.intake().whileTrue(m_intake.run(() -> m_intake.setIntake(3)));
        operatorControl.outtake().whileTrue(m_intake.run(() -> m_intake.setIntake(-3)));


        // TODO: spin up the shooter whenever the robot is in range.
        m_shooter.setDefaultCommand(m_shooter.run(() -> m_shooter.setVelocity(0.0)));
        operatorControl.shooter().whileTrue(m_shooter.run(() -> m_shooter.setVelocity(1)));

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
        m_indexer.setDefaultCommand(m_indexer.run(() -> m_indexer.setDrive(0)));
        operatorControl.index().whileTrue(m_indexer.run(() -> m_indexer.setDrive(3.5)));

        // TODO: presets
        // m_climber.setDefaultCommand(m_climber.run(() -> m_climber.set(operatorControl.climberState())));
        m_amp.setDefaultCommand(m_pivotAmp);

        ///////////////////////////
        //
        // DRIVE
        //

        PIDController thetaController = new PIDController(1.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController omegaController = new PIDController(0.5, 0, 0);

        m_drive.setDefaultCommand(
                new DriveManually(
                        manualMode,
                        driverControl::twist,
                        m_drive,
                        m_heading,
                        swerveKinodynamics,
                        driverControl::desiredRotation,
                        thetaController,
                        omegaController,
                        driverControl::target,
                        driverControl::trigger));

        m_auton = m_drive.runInit(m_drive::defense);
        // selftest uses fields we just initialized above, so it comes last.
        m_selfTest = new SelfTestRunner(this, operatorControl::selfTestEnable);
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

    @Override
    public MorseCodeBeep getBeep() {
        return m_beep;
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
        m_autonSelector.close();
        m_allianceSelector.close();
        m_indicator.close();
        m_modules.close();
    }

    //////////////////////////////////
    //
    // for testing

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return m_drive;
    }

    @Override
    public Command getDrawCircle() {
        return m_drawCircle;
    }

    @Override
    public DriveInALittleSquare getDriveInALittleSquare() {
        return m_driveInALittleSquare;
    }

    @Override
    public Monitor getMonitor() {
        return m_monitor;
    }

    @Override
    public HeadingInterface getHeading() {
        return m_heading;
    }
}
