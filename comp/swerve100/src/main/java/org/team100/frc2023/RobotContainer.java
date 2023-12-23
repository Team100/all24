package org.team100.frc2023;

import java.io.IOException;
import java.util.List;

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
import org.team100.lib.config.AllianceSelector;
import org.team100.lib.config.AutonSelector;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.controller.DrivePursuitController;
import org.team100.lib.controller.DriveRamseteController;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.experiments.Experiments;
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
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SpeedLimitsFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.SwerveModuleCollectionFactory;
import org.team100.lib.motion.drivetrain.SwerveModuleCollectionInterface;
import org.team100.lib.motion.drivetrain.SwerveModuleFactory;
import org.team100.lib.motion.drivetrain.VeeringCorrection;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.motion.drivetrain.kinematics.SwerveDriveKinematicsFactory;
import org.team100.lib.motion.simple.SimpleSubsystem;
import org.team100.lib.motion.simple.SimpleSubsystemFactory;
import org.team100.lib.selftest.Testable;
import org.team100.lib.sensors.HeadingFactory;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.swerve.SwerveKinematicLimits;
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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer implements Testable {

    //////////////////////////////////////
    // SHOW MODE
    //
    // Show mode is for younger drivers to drive the robot slowly.
    //
    // TODO: make a physical show mode switch.
    // TODO: make way more noticable.
    private static final boolean SHOW_MODE = false;
    //
    //////////////////////////////////////

    private static final double kDriveCurrentLimit = 30;
    // public double kDriveCurrentLimit = SHOW_MODE ? 20 : 60;

    private final Telemetry t = Telemetry.get();

    private final AutonSelector m_autonSelector;
    private final AllianceSelector m_allianceSelector;

    private final HeadingInterface m_heading;
    private final LEDIndicator m_indicator;
    private final AprilTagFieldLayoutWithCorrectOrientation layout;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveModuleCollectionInterface m_modules;
    private final SwerveDriveKinematics m_kinematics;
    private final Command m_auton;
    private final FrameTransform m_frameTransform;

    private final DriverControl control;
    private final DrawCircle m_drawCircle;
    private final Monitor m_monitor;

    // Identity-specific fields
    private final ArmSubsystem m_armSubsystem;
    private final ArmKinematics m_armKinematicsM;

    private final SimpleSubsystem m_elevator;

    public RobotContainer(TimedRobot robot) throws IOException {

        m_autonSelector = new AutonSelector();
        t.log(Level.INFO, "/Routine", getRoutine());

        m_allianceSelector = new AllianceSelector();
        t.log(Level.INFO, "/Alliance", m_allianceSelector.alliance().name());

        m_indicator = new LEDIndicator(8);

        m_monitor = new Monitor(new Annunciator(0));
        robot.addPeriodic(m_monitor::periodic, 0.02);

        Identity identity = Identity.get();

        m_kinematics = SwerveDriveKinematicsFactory.get(identity);
        Experiments experiments = new Experiments(identity);

        SwerveModuleFactory moduleFactory = new SwerveModuleFactory(experiments, kDriveCurrentLimit);
        m_modules = new SwerveModuleCollectionFactory(identity, moduleFactory).get();

        m_heading = HeadingFactory.get(identity, m_kinematics, m_modules);

        SpeedLimits speedLimits = SpeedLimitsFactory.get(identity, SHOW_MODE);

        // TODO replace with SpeedLimits.
        // TODO: fix these limits
        SwerveKinematicLimits swerveKinematicLimits = new SwerveKinematicLimits(4, 2, 2, 13, 5);

        VeeringCorrection veering = new VeeringCorrection(m_heading::getHeadingRateNWU);

        m_frameTransform = new FrameTransform(veering);

        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.1, 0.1, 0.4));

        // TODO: make this override work better
        // if (m_allianceSelector.alliance() == DriverStation.Alliance.Blue) {
        layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout("2023-studies.json");
        // } else { // red
        // layout =
        // AprilTagFieldLayoutWithCorrectOrientation.redLayout("2023-studies.json");
        // }

        VisionDataProvider visionDataProvider = new VisionDataProvider(
                layout,
                poseEstimator,
                poseEstimator::getEstimatedPosition);
        visionDataProvider.enable();

        SwerveLocal swerveLocal = new SwerveLocal(
                experiments,
                speedLimits,
                m_kinematics,
                m_modules);

        // control = new JoystickControl();
        // control = new DriverXboxControl();

        // selects the correct control class for whatever is plugged in
        control = new ControlFactory().getDriverControl();

        m_drive = new SwerveDriveSubsystem(
                m_heading,
                poseEstimator,
                m_frameTransform,
                swerveLocal,
                control::speed);

        ////////////////////////////
        //
        // DRIVETRAIN COMMANDS
        //

        control.defense().whileTrue(m_drive.runInit(m_drive::defense));
        control.steer0().whileTrue(m_drive.runInit(m_drive::steer0));
        control.steer90().whileTrue(m_drive.runInit(m_drive::steer90));

        control.resetRotation0().onTrue(new SetRotation(m_drive, GeometryUtil.kRotationZero));
        control.resetRotation180().onTrue(new SetRotation(m_drive, Rotation2d.fromDegrees(180)));

        // TODO: use the same controllers as HolonomicDriveController3
        // P is low here to avoid oscillating
        // TODO: add a velocity control
        PIDController thetaController = new PIDController(2, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        ManualMode manualMode = new ManualMode();

        // slow/medium mode are now modifiers in SwerveDriveSubsystem.
        // SpeedLimits slow = new SpeedLimits(0.4, 1.0, 0.5, 1.0);
        // control.driveSlow().whileTrue(
        // new DriveManually(manualMode,
        // control::twist,
        // m_drive,
        // m_heading,
        // slow,
        // control::desiredRotation,
        // thetaController));
        // SpeedLimits medium = new SpeedLimits(2.0, 2.0, 0.5, 1.0);
        // control.driveMedium().whileTrue(
        // new DriveManually(manualMode,
        // control::twist,
        // m_drive,
        // m_heading,
        // medium,
        // control::desiredRotation,
        // thetaController));

        // TODO: make the reset configurable
        // control.resetPose(new ResetPose(m_robotDrive, 0, 0, 0));
        control.resetPose().onTrue(new ResetPose(m_drive, 0, 0, Math.PI));

        HolonomicDriveController3 controller = new HolonomicDriveController3();

        control.rotate0().whileTrue(new Rotate(m_drive, m_heading, speedLimits, 0));

        m_drawCircle = new DrawCircle(experiments, m_drive, m_kinematics, controller);
        control.circle().whileTrue(m_drawCircle);

        TrajectoryPlanner planner = new TrajectoryPlanner(m_kinematics, swerveKinematicLimits);

        control.driveWithFancyTrajec().whileTrue(
                new FancyTrajectory(m_kinematics, swerveKinematicLimits, m_drive, planner));

        control.never().whileTrue(new DriveInACircle(m_drive, controller, -1));
        control.never().whileTrue(new Spin(m_drive, controller));
        control.never().whileTrue(new Oscillate(experiments, m_drive));

        // make a one-meter line
        control.never().whileTrue(
                new TrajectoryListCommand(m_drive, controller,
                        x -> List.of(TrajectoryMaker.line(m_kinematics, x))));

        // make a one-meter square
        control.never().whileTrue(
                new TrajectoryListCommand(m_drive, controller,
                        x -> TrajectoryMaker.square(m_kinematics, x)));

        // one-meter square with reset at the corners
        control.actualCircle().whileTrue(
                new PermissiveTrajectoryListCommand(m_drive, controller,
                        TrajectoryMaker.permissiveSquare(m_kinematics)));

        // one-meter square with position and velocity feedback control
        control.never().whileTrue(
                new FullStateTrajectoryListCommand(m_drive,
                        x -> TrajectoryMaker.square(m_kinematics, x)));

        // trying the new ChoreoLib
        ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory("test");
        control.never().whileTrue(CommandMaker.choreo(choreoTrajectory, m_drive));

        // playing with trajectory followers
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        StraightLineTrajectory maker = new StraightLineTrajectory(experiments, config);
        Pose2d goal = new Pose2d(8, 4, new Rotation2d()); // field center, roughly
        Command follower = new DriveToWaypoint3(goal, m_drive, maker, controller);
        control.never().whileTrue(follower);

        // 254 PID follower
        DriveMotionController drivePID = new DrivePIDFController(false);
        control.never().whileTrue(
                new DriveToWaypoint100(goal, m_drive, planner, drivePID));

        // 254 FF follower
        DriveMotionController driveFF = new DrivePIDFController(true);
        control.never().whileTrue(
                new DriveToWaypoint100(goal, m_drive, planner, driveFF));

        // 254 Pursuit follower
        DriveMotionController drivePP = new DrivePursuitController();
        control.never().whileTrue(
                new DriveToWaypoint100(goal, m_drive, planner, drivePP));

        // 254 Ramsete follower
        // this one seems to have a pretty high tolerance?
        DriveMotionController driveRam = new DriveRamseteController();
        control.never().whileTrue(
                new DriveToWaypoint100(goal, m_drive, planner, driveRam));

        // little square
        control.never().whileTrue(
                new DriveInALittleSquare(m_drive));

        ///////////////////////
        //
        // ARM
        //

        OperatorControl operatorControl = new ControlFactory().getOperatorControl();

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
        m_elevator = new SimpleSubsystemFactory(identity, experiments).get();
        SimpleManualMode simpleMode = new SimpleManualMode();
        m_elevator.setDefaultCommand(new SimpleManual(simpleMode, m_elevator, operatorControl::elevator));

        ///////////////////////////
        //
        // DRIVE
        //

        // if (m_config.SHOW_MODE) {

        m_drive.setDefaultCommand(
                new DriveManually(
                        manualMode,
                        control::twist,
                        m_drive,
                        m_heading,
                        speedLimits,
                        control::desiredRotation,
                        thetaController,
                        control::target,
                        control::trigger));

        // } else {
        // m_drive.setDefaultCommand(
        // new DriveWithHeading(
        // control::twist,
        // m_drive,
        // m_heading,
        // speedLimits,
        // control::desiredRotation,
        // thetaController));
        // }

        /////////////////////////////////
        //
        // IDENTITY-SPECIFIC PARTS
        //

        switch (identity) {
            case TEST_BOARD_6B:
                // TODO: use the correct identity.
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

    public Command getDrawCircle() {
        return m_drawCircle;
    }

    public Monitor getMonitor() {
        return m_monitor;
    }

    @Override
    public HeadingInterface getHeading() {
        return m_heading;
    }
}
