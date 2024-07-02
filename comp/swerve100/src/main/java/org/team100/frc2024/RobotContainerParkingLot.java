package org.team100.frc2024;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.commands.drivetrain.CommandMaker;
import org.team100.lib.commands.drivetrain.DrawSquare;
import org.team100.lib.commands.drivetrain.DriveInACircle;
import org.team100.lib.commands.drivetrain.DriveInALittleSquare;
import org.team100.lib.commands.drivetrain.DriveToState101;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.commands.drivetrain.FullStateTrajectoryListCommand;
import org.team100.lib.commands.drivetrain.Oscillate;
import org.team100.lib.commands.drivetrain.PermissiveTrajectoryListCommand;
import org.team100.lib.commands.drivetrain.Spin;
import org.team100.lib.commands.drivetrain.TrajectoryListCommand;
import org.team100.lib.commands.telemetry.MorseCodeBeep;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DriveMotionControllerFactory;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.HeadingFactory;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Annunciator;
import org.team100.lib.telemetry.Monitor;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.TrajectoryMaker;
import org.team100.lib.visualization.SwerveModuleVisualization;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is stuff cut out of RobotContainer, so that the compiler will still see
 * it, but so that it's not in the prod robot execution path at all.
 */
public class RobotContainerParkingLot {
    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    private static final double kDriveCurrentLimit = 60;
    private static final double kDriveStatorLimit = 120;

    private final SwerveModuleCollection m_modules;
    final HeadingInterface m_heading;
    final SwerveDriveSubsystem m_drive;
    final DriverControl driverControl;
    final OperatorControl operatorControl;

    /**
     * Stuff to come back to someday.
     * 
     * The reason to put it here rather than commenting it out is so that it doesn't
     * rot.
     */
    RobotContainerParkingLot(TimedRobot100 robot) {
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        driverControl = new DriverControlProxy(async);
        operatorControl = new OperatorControlProxy(async);
        final SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();
        m_modules = SwerveModuleCollection.get(
                kDriveCurrentLimit,
                kDriveStatorLimit,
                swerveKinodynamics,
                async);
        SwerveModuleVisualization.make(m_modules, async);
        m_heading = HeadingFactory.get(
                swerveKinodynamics,
                m_modules,
                asyncFactory);
        SwerveDrivePoseEstimator100 poseEstimator = swerveKinodynamics.newPoseEstimator(
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                Timer.getFPGATimestamp(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE)); // 0.1 0.1
        SwerveLocal swerveLocal = new SwerveLocal(swerveKinodynamics, m_modules);
        m_drive = new SwerveDriveSubsystem(
                m_heading,
                poseEstimator,
                swerveLocal,
                driverControl::speed);

        // joel 2/22/24 removing for SVR, put back after that.
        // these should be fields
        final MorseCodeBeep m_beep;
        final Monitor m_monitor;

        // joel 2/22/24 removing for SVR, put it back after that.
        // 20 words per minute is 60 ms.
        m_beep = new MorseCodeBeep(0.06);
        // m_beep = new Beep();
        BooleanSupplier test = () -> driverControl.annunicatorTest() ||
                m_beep.getOutput();
        m_monitor = new Monitor(new Annunciator(6, async), test);
        // The monitor runs less frequently than the control loop.
        robot.addPeriodic(m_monitor::periodic, 0.2, "monitor");

        HolonomicDriveController3 controller = new HolonomicDriveController3();

        ///////////////////////

        // little square
        // this should be a field.
        final DriveInALittleSquare m_driveInALittleSquare;

        m_driveInALittleSquare = new DriveInALittleSquare(m_drive);
        whileTrue(driverControl::never, m_driveInALittleSquare);

        ///////////////////////
        // trying the new ChoreoLib
        ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory("test");
        whileTrue(driverControl::never, CommandMaker.choreo(choreoTrajectory, m_drive));

        ///////////////////////

        whileTrue(driverControl::never, new DriveInACircle(m_drive, controller, -1));
        whileTrue(driverControl::never, new Spin(m_drive, controller));
        whileTrue(driverControl::never, new Oscillate(m_drive));

        ////////////////////////

        // field center, roughly, facing to the left.
        Pose2d goal = new Pose2d(1.877866, 7.749999, GeometryUtil.kRotation90);
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();

        // 254 PID follower
        DriveMotionController drivePID = DriveMotionControllerFactory.autoPIDF();
        whileTrue(driverControl::never,
                new DriveToWaypoint100(
                        goal,
                        m_drive,
                        drivePID,
                        constraints,
                        1));

        ////////////////////////

        // 254 FF follower

        DriveMotionController driveFF = DriveMotionControllerFactory.ffOnly();

        whileTrue(driverControl::never,
                new DriveToWaypoint100(
                        goal,
                        m_drive,
                        driveFF,
                        constraints,
                        1));

        ///////////////////////

        // 254 Pursuit follower
        DriveMotionController drivePP = DriveMotionControllerFactory.purePursuit(swerveKinodynamics);

        whileTrue(driverControl::test,
                new DriveToWaypoint100(
                        goal,
                        m_drive,
                        drivePP,
                        constraints,
                        1));

        whileTrue(driverControl::test,
                new DriveToState101(
                        goal,
                        new FieldRelativeVelocity(2, 0, 0),
                        m_drive,
                        drivePP,
                        constraints));

        ///////////////////////

        // 254 Ramsete follower
        // this one seems to have a pretty high tolerance?
        DriveMotionController driveRam = DriveMotionControllerFactory.ramsete();
        whileTrue(driverControl::never,
                new DriveToWaypoint100(
                        goal,
                        m_drive,
                        driveRam,
                        constraints,
                        1));

        //////////////////////

        // calibration

        TrajectoryMaker maker = new TrajectoryMaker(constraints);

        // make a one-meter line
        whileTrue(driverControl::never,
                new TrajectoryListCommand(m_drive, controller,
                        x -> List.of(maker.line(x))));

        // make a one-meter square
        whileTrue(driverControl::never,
                new TrajectoryListCommand(m_drive, controller,
                        maker::square));

        whileTrue(driverControl::test, new TrajectoryListCommand(m_drive, controller,
                null));

        // one-meter square with reset at the corners
        whileTrue(driverControl::never,
                new PermissiveTrajectoryListCommand(m_drive, controller,
                        maker.permissiveSquare()));

        // one-meter square with position and velocity feedback control
        whileTrue(driverControl::never,
                new FullStateTrajectoryListCommand(m_drive,
                        maker::square));

        // this should be a field.
        final DrawSquare m_drawCircle = new DrawSquare(m_drive, controller);
        whileTrue(driverControl::circle, m_drawCircle);
    }

    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

}
