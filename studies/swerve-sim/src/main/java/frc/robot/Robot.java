
package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.team100.frc2023.autonomous.Autonomous;
import org.team100.frc2023.commands.DriveManually;
import org.team100.frc2023.commands.ResetRotation;
import org.team100.frc2023.control.LogitechExtreme3dControl;
import org.team100.frc2023.control.ManualControl;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
import org.team100.lib.controller.HolonomicDriveController2;
import org.team100.lib.motion.drivetrain.VeeringCorrection;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import util.PinkNoise;

public class Robot extends TimedRobot {
    private final ManualControl m_manualControl;
    private final Drivetrain m_swerve;
    private final Command m_driveCommand;
    // private final Command m_drivePositional;

    Command autoc;
    ProfiledPIDController m_rotationController;
    PIDController xController;
    PIDController yController;

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

    VeeringCorrection veering;

    FrameTransform m_frameTransform;

    public Robot() {
        final AnalogGyro gyro = new AnalogGyro(0);
        DriveControllers controllers = new DriveControllersFactory().get();
        HolonomicDriveController2 controller = new HolonomicDriveController2(controllers);
        veering = new VeeringCorrection(() -> -1.0 * gyro.getRate());
        m_frameTransform = new FrameTransform(veering);
        // alpha = 1.5 => between "pink" and random-walk "brownian"
        m_swerve = new Drivetrain(gyro,
                () -> new PinkNoise(1.5, 3), controller, m_frameTransform);

        // m_manualControl = new XboxControl();
        m_manualControl = new LogitechExtreme3dControl();
        // m_manualControl = new Pilot();
        m_manualControl.resetRotation0(new ResetRotation(m_swerve, new Rotation2d(0)));
        m_driveCommand = new DriveManually(m_swerve, m_manualControl::xSpeed,
                m_manualControl::ySpeed,
                m_manualControl::rotSpeed);
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
        Command waypointCommand = toWaypoint2();
        m_manualControl.topButton().whileTrue(waypointCommand);
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
        // autoc = auto();
        // autoc = circle();
        // autoc = toWaypoint();
        // autoc = toWaypoint2();
        autoc = new WrapperCommand(
                new Autonomous(m_swerve, m_frameTransform, m_swerve.m_gyro, 2)) {
            @Override
            public void execute() {
                System.out.println("wrapper execute");
                m_command.execute();
            }

            @Override
            public void end(boolean interrupted) {
                System.out.println("wrapper end");
                m_command.end(interrupted);
            }
        };
        autoc.schedule();
    }

    @Override
    public void autonomousPeriodic() {
        // driveWithJoystick(false);
        // m_swerve.updateOdometry();

        // System.out.printf("scheduled %s\n", autoc.isScheduled()?"yes":"no");

        // TODO move this to a class for the circle path etc.
        // m_rotationPositionError.set(m_rotationController.getPositionError());
        // m_rotationVelocityError.set(m_rotationController.getVelocityError());
        // m_rotationSetpointPosition.set(m_rotationController.getSetpoint().position);
        // m_rotationSetpointVelocity.set(m_rotationController.getSetpoint().velocity);

        // observe the controllers

        // m_XErrorPub.set(xController.getPositionError());
        // m_YErrorPub.set(yController.getPositionError());

    }

    @Override
    public void autonomousExit() {
        if (autoc != null)
            autoc.cancel();
    }

    public Command toWaypoint() {
        // fixed waypoint for now
        // Supplier<Pose2d> waypointSupplier = () -> new Pose2d(8, 4, new
        // Rotation2d(-Math.PI / 2));
        Supplier<Pose2d> waypointSupplier = () -> new Pose2d(8, 0, new Rotation2d());
        Supplier<Pose2d> poseSupplier = m_swerve::getPose;
        Consumer<SwerveModuleState[]> outputModuleStates = m_swerve::setModuleStates;
        return new DriveToWaypoint(waypointSupplier, poseSupplier, m_swerve.m_kinematics,
                outputModuleStates, m_swerve);
    }

    public Command toWaypoint2() {
        return new DriveToWaypoint2(new Pose2d(8, 4, new Rotation2d()), m_swerve);
    }

    /**
     * Make a circular path with a single spline
     */
    public Command circle() {
        TrajectoryConfig translationConfig = new TrajectoryConfig(Drivetrain.kMaxSpeed / 2, Drivetrain.kMaxSpeed)
                .setKinematics(m_swerve.m_kinematics);
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
        xController = new PIDController(5, 0, 0);
        yController = new PIDController(5, 0, 0);

        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
                Drivetrain.kMaxAngularSpeed / 2, Drivetrain.kMaxAngularSpeed / 2);
        // OK go a little crazy with the PID
        // m_rotationController = new ProfiledPIDController(1.5, 0, 0,
        // rotationConstraints);
        m_rotationController = new ProfiledPIDController(5, 0, 0, rotationConstraints);
        SmartDashboard.putData("rotation controller", m_rotationController);

        SwerveControllerCommand swerveControllerCommand0 = new SwerveControllerCommand(target0,
                m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
                () -> new Rotation2d(0), // no robot rotation at all
                m_swerve::setModuleStates, m_swerve);

        return swerveControllerCommand0.andThen(() -> m_swerve.truncate());
    }

    /**
     * Make a zigzag path, imagine these were task stations
     */
    public Command auto() {
        TrajectoryConfig translationConfig = new TrajectoryConfig(Drivetrain.kMaxSpeed / 2, Drivetrain.kMaxSpeed)
                .setKinematics(m_swerve.m_kinematics);

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

        xController = new PIDController(1.5, 0, 0);
        yController = new PIDController(1.5, 0, 0);
        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
                Drivetrain.kMaxAngularSpeed / 2, Drivetrain.kMaxAngularSpeed / 2);
        m_rotationController = new ProfiledPIDController(1.5, 0, 0, rotationConstraints);
        SmartDashboard.putData("rotation controler", m_rotationController);

        // if you don't reset the pose, it will kinda do the right thing, trying to get
        // to the correct place no matter where it starts.
        // m_swerve.resetOdometry(target0.getInitialPose());

        // specify the rotations here, otherwise it takes the last spline control,
        // which only makes sense for tank drive.
        SwerveControllerCommand swerveControllerCommand0 = new SwerveControllerCommand(target0,
                m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
                () -> new Rotation2d(Math.PI / 2),
                m_swerve::setModuleStates, m_swerve);
        SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(target1,
                m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
                () -> new Rotation2d(-Math.PI / 2),
                m_swerve::setModuleStates, m_swerve);
        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(target2,
                m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
                () -> new Rotation2d(Math.PI / 2),
                m_swerve::setModuleStates, m_swerve);
        SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(target3,
                m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
                () -> new Rotation2d(-Math.PI / 2),
                m_swerve::setModuleStates, m_swerve);

        // run the sequence and stop at the end. added pauses for some realism, e.g.
        // like the robot is doing something.
        return swerveControllerCommand0.andThen(new WaitCommand(0.1))
                .andThen(swerveControllerCommand1).andThen(new WaitCommand(0.1))
                .andThen(swerveControllerCommand2).andThen(new WaitCommand(0.1))
                .andThen(swerveControllerCommand3).andThen(new WaitCommand(0.1))
                .andThen(() -> m_swerve.truncate());
    }

    @Override
    public void teleopInit() {
        System.out.println("teleop init");
        super.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        // driveWithJoystick(true);
    }

    @Override
    public void simulationInit() {
        m_swerve.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        m_swerve.simulationPeriodic();
        m_swerve.updateOdometry();
    }

    @Override
    public void robotPeriodic() {
        // m_swerve.updateOdometry();
        // if you forget this scheduler thing then nothing will happen
        CommandScheduler.getInstance().run();
        System.out.printf("teleop %b, drive scheduled %b, default %s\n",
                isTeleopEnabled(),
                CommandScheduler.getInstance().isScheduled(m_driveCommand),
                m_swerve.getDefaultCommand().getName());
    }

    @Override
    public void testPeriodic() {
        // m_swerve.test(getYSpeedInput1_1(), getRotSpeedInput1_1());
    }
}
