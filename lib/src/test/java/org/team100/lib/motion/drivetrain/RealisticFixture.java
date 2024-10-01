package org.team100.lib.motion.drivetrain;

import org.team100.lib.controller.drivetrain.HolonomicDriveControllerFactory;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.localization.VisionData;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestSupplierLogger;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.SimulatedGyro;

/**
 * A real swerve subsystem populated with simulated motors and encoders,
 * for testing.
 */
public class RealisticFixture {
    public SwerveModuleCollection collection;
    public Gyro gyro;
    public SwerveDrivePoseEstimator100 poseEstimator;
    public SwerveKinodynamics swerveKinodynamics;
    public SwerveLocal swerveLocal;
    public SwerveDriveSubsystem drive;
    public HolonomicFieldRelativeController controller;
    public LoggerFactory logger;
    public LoggerFactory fieldLogger;

    public RealisticFixture() {
        logger = new TestSupplierLogger(new TestPrimitiveLogger());
        fieldLogger = new TestSupplierLogger(new TestPrimitiveLogger());
        swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        collection = SwerveModuleCollection.get(logger, 10, 20, swerveKinodynamics);
        gyro = new SimulatedGyro(swerveKinodynamics, collection);
        swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);
        poseEstimator = swerveKinodynamics.newPoseEstimator(
                logger,
                gyro.getYawNWU(),
                collection.positions(),
                GeometryUtil.kPoseZero,
                0); // initial time is zero here for testing
        VisionData v = new VisionData() {
            @Override
            public void update() {
            }
        };

        drive = new SwerveDriveSubsystem(
                fieldLogger,
                logger,
                gyro,
                poseEstimator,
                swerveLocal,
                v);

        controller = HolonomicDriveControllerFactory.get(
                new HolonomicFieldRelativeController.Log(logger));
    }

    public void close() {
        // close the DIO inside the turning encoder
        collection.close();
    }

}
