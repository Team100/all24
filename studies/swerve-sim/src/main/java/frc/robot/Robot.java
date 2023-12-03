
package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.Line;
import org.team100.lib.commands.drivetrain.ManualMode.Mode;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.commands.drivetrain.TrajectoryCommand;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.JoystickControl;
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
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.sensors.SimulatedHeading;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Robot extends TimedRobot {
    private final DriverControl m_manualControl;
    private final SwerveDriveSubsystem m_swerve;
    private final Command m_driveCommand;
    // private final Command m_drivePositional;

    Command autoc;
    // ProfiledPIDController m_rotationController;
    // PIDController xController;
    // PIDController yController;

    // private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // private final NetworkTable m_table = inst.getTable("robot");
    // private final DoublePublisher m_rotationSetpointPosition =
    // m_table.getDoubleTopic("rotationSetpointPosition")
    // .publish();
    // private final DoublePublisher m_rotationSetpointVelocity =
    // m_table.getDoubleTopic("rotationSetpointVelocity")
    // .publish();

    // private final DoublePublisher m_rotationPositionError =
    // m_table.getDoubleTopic("rotationPositionError").publish();
    // private final DoublePublisher m_rotationVelocityError =
    // m_table.getDoubleTopic("rotationVelocityError").publish();

    // // controller errors
    // private final DoublePublisher m_XErrorPub =
    // m_table.getDoubleTopic("xError").publish();
    // private final DoublePublisher m_YErrorPub =
    // m_table.getDoubleTopic("yError").publish();

    private final VeeringCorrection veering;
    private final FrameTransform m_frameTransform;
    private final SwerveDriveKinematics m_kinematics;
    private final SwerveModuleCollectionInterface m_modules;
    private final HeadingInterface m_heading;
    private final Field2d m_field;
    private final DriveControllers controllers;
    private final HolonomicDriveController3 m_controller3;

    Command waypointCommand;

    public Robot() {
        Identity identity = Identity.BLANK;
        m_kinematics = SwerveDriveKinematicsFactory.get(identity);
        Experiments experiments = new Experiments(identity);
        SwerveModuleFactory moduleFactory = new SwerveModuleFactory(experiments, 60);
        m_modules = new SwerveModuleCollectionFactory(identity, moduleFactory).get();

        m_heading = new SimulatedHeading(m_kinematics, m_modules);

        // final AnalogGyro gyro = new AnalogGyro(0);
        controllers = new DriveControllersFactory().get(identity);
        m_controller3 = new HolonomicDriveController3(controllers);
        m_controller3.setTolerance(0.1, 1.0);

        veering = new VeeringCorrection(() -> m_heading.getHeadingRateNWU());
        m_frameTransform = new FrameTransform(veering);
        // alpha = 1.5 => between "pink" and random-walk "brownian"
        // m_swerve = new Drivetrain(gyro,
        // () -> new PinkNoise(1.5, 3), controller, m_frameTransform);

        SpeedLimits speedLimits = SpeedLimitsFactory.get(identity, false);

        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.1, 0.1, 0.4));

        SwerveLocal swerveLocal = new SwerveLocal(
                experiments,
                speedLimits,
                m_kinematics,
                m_modules);

        m_field = new Field2d();

        m_swerve = new SwerveDriveSubsystem(
                m_heading,
                poseEstimator,
                m_frameTransform,
                swerveLocal,
                m_field

        );

        // m_manualControl = new XboxControl();
        m_manualControl = new JoystickControl();
        // m_manualControl = new Pilot();
        m_manualControl.resetRotation0().onTrue(new SetRotation(m_swerve, new Rotation2d(0)));
        m_driveCommand = new DriveManually(
                () -> Mode.FIELD_RELATIVE_TWIST,
                m_manualControl::twist,
                m_swerve,
                speedLimits);
        // m_driveCommand = new DriveWithHeading(
        // m_swerve,
        // m_manualControl::xSpeed,
        // m_manualControl::ySpeed,
        // m_manualControl::desiredRotation,
        // m_manualControl::rotSpeed);
        // m_drivePositional = new DrivePositional(
        // m_swerve,
        // m_manualControl::xSpeed,
        // m_manualControl::ySpeed,
        // m_manualControl::desiredRotation);
        waypointCommand = toWaypoint2();
        m_manualControl.circle().whileTrue(waypointCommand);
        // drive normally if the trigger is down but not the thumb
        // m_manualControl.trigger().and(m_manualControl.thumb().negate()).whileTrue(m_driveCommand);
        // drive positional if the thumb is down
        // m_manualControl.thumb().whileTrue(m_drivePositional);
        m_swerve.setDefaultCommand(m_driveCommand);
        // default is nothing
        // m_swerve.removeDefaultCommand();
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    public Command toWaypoint() {
        TrajectoryConfig config = new TrajectoryConfig(4, 2).setKinematics(m_kinematics);
        return  Line.line(new Pose2d(8, 0, new Rotation2d()), m_swerve, config);
    }

    public Command toWaypoint2() {
        TrajectoryConfig config = new TrajectoryConfig(4, 2).setKinematics(m_kinematics);
        return Line.line(new Pose2d(8, 4, new Rotation2d()), m_swerve, config);
    }

    /**
     * Make a circular path with a single spline
     */
    public Command circle() {
        TrajectoryConfig translationConfig = new TrajectoryConfig(Drivetrain.kMaxSpeed / 2, Drivetrain.kMaxSpeed)
                .setKinematics(m_kinematics);
        Trajectory target0 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(Math.PI / 4)), // start ~diagonally
                List.of( // make three loops
                        new Translation2d(5, 5), // kind of an insertion here
                        new Translation2d(8, 6),
                        new Translation2d(10, 4),
                        new Translation2d(8, 2),
                        new Translation2d(6, 4),
                        new Translation2d(8, 6),
                        new Translation2d(10, 4),
                        new Translation2d(8, 2),
                        new Translation2d(6, 4),
                        new Translation2d(8, 6),
                        new Translation2d(10, 4),
                        new Translation2d(8, 2)),
                new Pose2d(6, 4, new Rotation2d(Math.PI / 2)), // end +y
                translationConfig);
        // OK go a little crazy with the PID
        // xController = new PIDController(1.5, 0, 0);
        // yController = new PIDController(1.5, 0, 0);
        // xController = new PIDController(5, 0, 0);
        // yController = new PIDController(5, 0, 0);

        // TrapezoidProfile.Constraints rotationConstraints = new
        // TrapezoidProfile.Constraints(
        // Drivetrain.kMaxAngularSpeed / 2, Drivetrain.kMaxAngularSpeed / 2);
        // OK go a little crazy with the PID
        // m_rotationController = new ProfiledPIDController(1.5, 0, 0,
        // rotationConstraints);
        // m_rotationController = new ProfiledPIDController(5, 0, 0,
        // rotationConstraints);
        // SmartDashboard.putData("rotation controller", m_rotationController);

        TrajectoryCommand swerveControllerCommand0 = new TrajectoryCommand(target0, m_swerve, m_controller3);

        return swerveControllerCommand0.andThen(() -> m_swerve.stop());
    }

    /**
     * Make a zigzag path, imagine these were task stations
     */
    public Command auto() {
        TrajectoryConfig translationConfig = new TrajectoryConfig(Drivetrain.kMaxSpeed / 2, Drivetrain.kMaxSpeed)
                .setKinematics(m_kinematics);

        // for now just make straight lines.
        // remember the endpoint poses are not robot poses, they are **spline controls**
        Trajectory target0 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                new ArrayList<>(),
                new Pose2d(4, 7, new Rotation2d(Math.PI / 2)), translationConfig);

        Trajectory target1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(4, 7, new Rotation2d(-Math.PI / 2)),
                new ArrayList<>(),
                new Pose2d(8, 1, new Rotation2d(-Math.PI / 2)),
                translationConfig);

        Trajectory target2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(8, 1, new Rotation2d(Math.PI / 2)),
                new ArrayList<>(),
                new Pose2d(12, 7, new Rotation2d(Math.PI / 2)),
                translationConfig);

        Trajectory target3 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(12, 7, new Rotation2d(-Math.PI / 2)),
                new ArrayList<>(),
                new Pose2d(15, 1, new Rotation2d(-Math.PI / 2)),
                translationConfig);

        // xController = new PIDController(1.5, 0, 0);
        // yController = new PIDController(1.5, 0, 0);
        // TrapezoidProfile.Constraints rotationConstraints = new
        // TrapezoidProfile.Constraints(
        // Drivetrain.kMaxAngularSpeed / 2, Drivetrain.kMaxAngularSpeed / 2);
        // m_rotationController = new ProfiledPIDController(1.5, 0, 0,
        // rotationConstraints);
        // SmartDashboard.putData("rotation controler", m_rotationController);

        // if you don't reset the pose, it will kinda do the right thing, trying to get
        // to the correct place no matter where it starts.
        // m_swerve.resetOdometry(target0.getInitialPose());

        // specify the rotations here, otherwise it takes the last spline control,
        // which only makes sense for tank drive.
        TrajectoryCommand swerveControllerCommand0 = new TrajectoryCommand(target0, m_swerve, m_controller3);
        TrajectoryCommand swerveControllerCommand1 = new TrajectoryCommand(target1, m_swerve, m_controller3);
        TrajectoryCommand swerveControllerCommand2 = new TrajectoryCommand(target2, m_swerve, m_controller3);
        TrajectoryCommand swerveControllerCommand3 = new TrajectoryCommand(target3, m_swerve, m_controller3);

        // run the sequence and stop at the end. added pauses for some realism, e.g.
        // like the robot is doing something.
        return swerveControllerCommand0.andThen(new WaitCommand(0.1))
                .andThen(swerveControllerCommand1).andThen(new WaitCommand(0.1))
                .andThen(swerveControllerCommand2).andThen(new WaitCommand(0.1))
                .andThen(swerveControllerCommand3).andThen(new WaitCommand(0.1))
                .andThen(() -> m_swerve.stop());
    }

    @Override
    public void teleopInit() {
        System.out.println("teleop init");
        // super.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        // driveWithJoystick(true);
    }

    @Override
    public void simulationInit() {
        // m_swerve.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        m_swerve.simulationPeriodic();
        // m_swerve.updateOdometry();
    }

    @Override
    public void robotPeriodic() {
        // m_swerve.updateOdometry();
        // if you forget this scheduler thing then nothing will happen
        CommandScheduler.getInstance().run();
        System.out.printf("teleop %b, drive scheduled %b, default %s wapyoint %b\n",
                isTeleopEnabled(),
                CommandScheduler.getInstance().isScheduled(m_driveCommand),
                m_swerve.getDefaultCommand().getName(),
                CommandScheduler.getInstance().isScheduled(waypointCommand));
    }

    @Override
    public void testPeriodic() {
        // m_swerve.test(getYSpeedInput1_1(), getRotSpeedInput1_1());
    }
}
