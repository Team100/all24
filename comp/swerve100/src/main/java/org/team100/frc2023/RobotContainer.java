package org.team100.frc2023;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.team100.lib.commands.arm.CartesianManualArm;
import org.team100.lib.commands.arm.CartesianManualPositionalArm;
import org.team100.lib.commands.arm.ManualArm;
import org.team100.lib.commands.arm.Sequence;
import org.team100.lib.commands.drivetrain.CommandMaker;
import org.team100.lib.commands.drivetrain.DrawCircle;
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
import org.team100.lib.commands.simple.SimpleManual;
import org.team100.lib.commands.simple.SimpleManualMode;
import org.team100.lib.commands.telemetry.MorseCodeBeep;
import org.team100.lib.config.AllianceSelector;
import org.team100.lib.config.AutonSelector;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.controller.DrivePursuitController;
import org.team100.lib.controller.DriveRamseteController;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.ControlFactory;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.VisionDataProvider;
import org.team100.lib.motion.arm.ArmFactory;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.motion.simple.SimpleSubsystem;
import org.team100.lib.motion.simple.SimpleSubsystemFactory;
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

    private final DrawCircle m_drawCircle;
    // for SelfTest
    private final DriveInALittleSquare m_driveInALittleSquare;
    private final MorseCodeBeep m_beep;
    private final Monitor m_monitor;

    // Identity-specific fields
    private final ArmSubsystem m_armSubsystem;
    private final ArmKinematics m_armKinematicsM;
    private final SimpleSubsystem m_elevator;

    public RobotContainer(TimedRobot robot) throws IOException {
        // selects the correct control class for whatever is plugged in
        ControlFactory controlFactory = new ControlFactory();
        DriverControl driverControl = controlFactory.getDriverControl();
        OperatorControl operatorControl = controlFactory.getOperatorControl();

        // digital inputs 0, 1, 2, 3.
        m_autonSelector = new AutonSelector();
        t.log(Level.INFO, "/Routine", getRoutine());

        // digital inputs 4, 5
        m_allianceSelector = new AllianceSelector();
        t.log(Level.INFO, "/Alliance", m_allianceSelector.alliance().name());

        m_indicator = new LEDIndicator(8);

        // 20 words per minute is 60 ms.
        m_beep = new MorseCodeBeep(0.06);
        // m_beep = new Beep();
        BooleanSupplier test = () -> driverControl.annunicatorTest() || m_beep.getOutput();
        // digital output 4
        m_monitor = new Monitor(new Annunciator(6), test);
        robot.addPeriodic(m_monitor::periodic, 0.02);

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

        SwerveLocal swerveLocal = new SwerveLocal(swerveKinodynamics, m_modules);

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

        m_drawCircle = new DrawCircle(m_drive, swerveKinodynamics, controller);
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

        ///////////////////////
        //
        // ARM
        //

        m_armSubsystem = ArmFactory.get();
        m_armKinematicsM = new ArmKinematics(0.93, 0.92);

        operatorControl.doSomething().whileTrue(new Sequence(m_armSubsystem, m_armKinematicsM));

        operatorControl.never().whileTrue(
                new ManualArm(
                        m_armSubsystem,
                        operatorControl::lower,
                        operatorControl::upper));

        operatorControl.never().whileTrue(
                new CartesianManualArm(
                        m_armSubsystem,
                        m_armKinematicsM,
                        operatorControl::lower,
                        operatorControl::upper));

        m_armSubsystem.setDefaultCommand(
                new CartesianManualPositionalArm(
                        m_armSubsystem,
                        m_armKinematicsM,
                        operatorControl::lower,
                        operatorControl::upper));

        ///////////////////////////
        //
        // ELEVATOR
        //
        m_elevator = new SimpleSubsystemFactory().get();
        SimpleManualMode simpleMode = new SimpleManualMode();
        m_elevator.setDefaultCommand(new SimpleManual(simpleMode, m_elevator, operatorControl::elevator));

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

        /////////////////////////////////
        //
        // IDENTITY-SPECIFIC PARTS
        //

        switch (Identity.instance) {
            case TEST_BOARD_6B:
                m_auton = new Sequence(m_armSubsystem, m_armKinematicsM);
                break;
            default:
                m_auton = m_drive.runInit(m_drive::defense);
                break;
        }
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
