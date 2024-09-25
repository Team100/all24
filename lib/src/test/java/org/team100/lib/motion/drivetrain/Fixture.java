package org.team100.lib.motion.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.localization.VisionData;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.SimulatedGyro;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.TestLogger;

/**
 * A real swerve subsystem populated with simulated motors and encoders,
 * for testing.
 */
public class Fixture {
    public SwerveModuleCollection collection;
    public Gyro gyro;
    public SwerveDrivePoseEstimator100 poseEstimator;
    public VeeringCorrection veering;
    public SwerveKinodynamics swerveKinodynamics;
    public SwerveLocal swerveLocal;
    public SwerveDriveSubsystem drive;
    public HolonomicDriveController3 controller;
    public SupplierLogger2 logger;
    public SupplierLogger2 fieldLogger;

    public Fixture() {
        logger = new TestLogger().getSupplierLogger();
        fieldLogger = new TestLogger().getSupplierLogger();
        swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
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

        controller = new HolonomicDriveController3(logger);
    }

    public void close() {
        // close the DIO inside the turning encoder
        collection.close();
    }

}
