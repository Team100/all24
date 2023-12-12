package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.SwerveModuleCollection;
import org.team100.lib.motion.drivetrain.SwerveModuleCollectionInterface;
import org.team100.lib.motion.drivetrain.SwerveModuleFactory;
import org.team100.lib.motion.drivetrain.VeeringCorrection;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.sensors.SimulatedHeading;
import org.team100.lib.trajectory.TrajectoryMaker;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

class TrajectoryListCommandTest {
    private static final double kDelta = 0.001;
    private static final double kDtS = 0.02;

    @Test
    void testSimple() {

        // instantiate a real drivetrain so that it really moves
        // TODO: extract this to some sort of test fixture

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(1, 1),
                new Translation2d(1, -1),
                new Translation2d(-1, 1),
                new Translation2d(-1, -1));
        Experiments experiments = new Experiments(Identity.BLANK);
        SwerveModuleFactory m_factory = new SwerveModuleFactory(experiments, 40);

        SwerveModuleCollectionInterface collection = new SwerveModuleCollection(
                m_factory.SimulatedModule("FrontLeft"),
                m_factory.SimulatedModule("FrontRight"),
                m_factory.SimulatedModule("RearLeft"),
                m_factory.SimulatedModule("RearRight"));
        HeadingInterface heading = new SimulatedHeading(kinematics, collection);
        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                heading.getHeadingNWU(),
                collection.positions(),
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.1, 0.1, 0.4));

        VeeringCorrection veering = new VeeringCorrection(heading::getHeadingRateNWU);

        FrameTransform m_frameTransform = new FrameTransform(veering);

        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        SwerveLocal swerveLocal = new SwerveLocal(
                experiments,
                speedLimits,
                kinematics,
                collection);

        Field2d field = new Field2d();

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(heading, poseEstimator,
                m_frameTransform, swerveLocal, field, () -> DriverControl.Speed.NORMAL);

        DriveControllers controllers = new DriveControllersFactory().get(Identity.BLANK);
        HolonomicDriveController3 control = new HolonomicDriveController3(controllers);

        TrajectoryListCommand c = new TrajectoryListCommand(
                drive,
                control,
                x -> List.of(TrajectoryMaker.line(kinematics, x)));
        c.initialize();
        assertEquals(0, drive.getPose().getX(), kDelta);
        c.execute();
        assertFalse(c.isFinished());
        // the trajectory takes about 2s
        for (double t = 0; t < 2; t += kDtS) {
            SimHooks.stepTiming(kDtS);
            c.execute();
            drive.periodic(); // for updateOdometry
        }
        // at goal within 5 mm
        assertEquals(1, drive.getPose().getX(), 0.005);
        assertTrue(c.isFinished());
    }
}
