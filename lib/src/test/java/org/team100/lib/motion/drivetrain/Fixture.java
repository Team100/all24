package org.team100.lib.motion.drivetrain;

import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.sensors.SimulatedHeading;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

/**
 * A real swerve subsystem populated with simulated motors and encoders,
 * for testing.
 */
public class Fixture {

    public Experiments experiments;
    public SwerveModuleFactory m_factory;
    public SwerveModuleCollectionInterface collection;
    public HeadingInterface heading;
    public SwerveDrivePoseEstimator poseEstimator;
    public VeeringCorrection veering;
    public FrameTransform m_frameTransform;
    public SwerveKinodynamics swerveKinodynamics;
    public SwerveLocal swerveLocal;
    public SwerveDriveSubsystem drive;

    public Fixture() {
        swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        
        experiments = new Experiments(Identity.BLANK);
        m_factory = new SwerveModuleFactory(experiments, 40);

        collection = new SwerveModuleCollection(
                m_factory.SimulatedModule("FrontLeft"),
                m_factory.SimulatedModule("FrontRight"),
                m_factory.SimulatedModule("RearLeft"),
                m_factory.SimulatedModule("RearRight"));
        heading = new SimulatedHeading(swerveKinodynamics.getKinematics(), collection);
        poseEstimator = new SwerveDrivePoseEstimator(
                swerveKinodynamics.getKinematics(),
                heading.getHeadingNWU(),
                collection.positions(),
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.1, 0.1, 0.4));

        veering = new VeeringCorrection(heading::getHeadingRateNWU);

        m_frameTransform = new FrameTransform(veering);

        swerveLocal = new SwerveLocal(
                experiments,
                swerveKinodynamics,
                collection);

        drive = new SwerveDriveSubsystem(heading, poseEstimator,
                m_frameTransform, swerveLocal, () -> DriverControl.Speed.NORMAL);

    }

}
