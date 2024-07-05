package org.team100.lib.motion.drivetrain;

import org.team100.lib.async.Async;
import org.team100.lib.async.MockAsync;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.sensors.SimulatedHeading;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.visualization.SwerveModuleVisualization;

import edu.wpi.first.math.VecBuilder;

/**
 * A real swerve subsystem populated with simulated motors and encoders,
 * for testing.
 */
public class Fixture {
    public SwerveModuleCollection collection;
    public HeadingInterface heading;
    public SwerveDrivePoseEstimator100 poseEstimator;
    public VeeringCorrection veering;
    public SwerveKinodynamics swerveKinodynamics;
    public SwerveLocal swerveLocal;
    public SwerveDriveSubsystem drive;
    public HolonomicDriveController3 controller;
    public Logger logger;

    public Fixture() {
        logger = new TestLogger();
        swerveKinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        Async async = new MockAsync();
        collection = SwerveModuleCollection.get(logger, 10, 20, swerveKinodynamics);
        SwerveModuleVisualization.make(collection, async);
        heading = new SimulatedHeading(swerveKinodynamics, collection);
        poseEstimator = swerveKinodynamics.newPoseEstimator(
                heading.getHeadingNWU(),
                collection.positions(),
                GeometryUtil.kPoseZero,
                0, // initial time is zero here for testing
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.1, 0.1, 0.4));

        swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);

        drive = new SwerveDriveSubsystem(
                logger,
                heading,
                poseEstimator,
                swerveLocal,
                () -> DriverControl.Speed.NORMAL);

        controller = new HolonomicDriveController3(logger);
    }

    public void close() {
        // close the DIO inside the turning encoder
        collection.close();
    }

}
