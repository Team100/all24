package org.team100.lib.motion.drivetrain;

import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.sensors.SimulatedHeading;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Fixture {

    /**
     * Make a real swerve subsystem populated with simulated motors and encoders,
     * for testing.
     */
    public static SwerveDriveSubsystem realSwerve(SwerveDriveKinematics kinematics) {

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

        return drive;

    }

}
