package org.team100.lib.motion.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.SimulatedGyro;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.TestLogger;

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
    public SupplierLogger logger;
    public SupplierLogger fieldLogger;

    public Fixture() {
        logger = new TestLogger().getSupplierLogger();
        fieldLogger = new TestLogger().getSupplierLogger();
        swerveKinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        collection = SwerveModuleCollection.get(logger, 10, 20, swerveKinodynamics);
        gyro = new SimulatedGyro(swerveKinodynamics, collection);
        poseEstimator = swerveKinodynamics.newPoseEstimator(
                gyro.getYawNWU(),
                collection.positions(),
                GeometryUtil.kPoseZero,
                0); // initial time is zero here for testing

        swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);

        drive = new SwerveDriveSubsystem(
                fieldLogger,
                logger,
                gyro,
                poseEstimator,
                swerveLocal);

        controller = new HolonomicDriveController3(logger);
    }

    public void close() {
        // close the DIO inside the turning encoder
        collection.close();
    }

}
