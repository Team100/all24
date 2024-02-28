package org.team100.lib.motion.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.copies.SwerveDrivePoseEstimator100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.sensors.SimulatedHeading;

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

    public Fixture() {
        swerveKinodynamics = SwerveKinodynamicsFactory.forTest();

        collection = SwerveModuleCollection.get(10, swerveKinodynamics);
        heading = new SimulatedHeading(swerveKinodynamics, collection);
        poseEstimator = swerveKinodynamics.newPoseEstimator(
                heading.getHeadingNWU(),
                collection.positions(),
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.1, 0.1, 0.4));

        swerveLocal = new SwerveLocal(swerveKinodynamics, collection);

        drive = new SwerveDriveSubsystem(
                heading,
                poseEstimator,
                swerveLocal,
                () -> DriverControl.Speed.NORMAL);

        controller = new HolonomicDriveController3();
    }

    public void close() {
        // close the DIO inside the turning encoder
        collection.close();
    }

}
