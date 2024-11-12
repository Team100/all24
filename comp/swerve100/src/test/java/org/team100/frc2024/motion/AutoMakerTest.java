package org.team100.frc2024.motion;

import org.junit.jupiter.api.Test;
import org.team100.frc2024.MockSensors;
import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.follower.DrivePIDFFollower;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.follower.DriveTrajectoryFollowerFactory;
import org.team100.lib.follower.DriveTrajectoryFollowerUtil;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.localization.VisionData;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.SimulatedGyro;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Tests construction of autons we wrote in 2024. */
class AutoMakerTest {
    public SwerveModuleCollection collection;
    public Gyro gyro;
    public SwerveDrivePoseEstimator100 poseEstimator;
    public SwerveKinodynamics swerveKinodynamics;
    public SwerveLocal swerveLocal;
    public SwerveDriveSubsystem drive;
    public HolonomicFieldRelativeController controller;
    public LoggerFactory logger;
    public LoggerFactory fieldLogger;

    @Test
    void testAll() {

        logger = new TestLoggerFactory(new TestPrimitiveLogger());
        fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        collection = SwerveModuleCollection.get(logger, 10, 20, swerveKinodynamics);
        gyro = new SimulatedGyro(swerveKinodynamics, collection);
        final AsymSwerveSetpointGenerator setpointGenerator = new AsymSwerveSetpointGenerator(
                logger,
                swerveKinodynamics,
                () -> 12);
        swerveLocal = new SwerveLocal(logger, swerveKinodynamics, setpointGenerator, collection);
        poseEstimator = swerveKinodynamics.newPoseEstimator(
                logger,
                gyro,
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

        final DriveTrajectoryFollowerUtil util = new DriveTrajectoryFollowerUtil(logger);

        final DriveTrajectoryFollowerFactory driveControllerFactory = new DriveTrajectoryFollowerFactory(util);
        DrivePIDFFollower.Log PIDFlog = new DrivePIDFFollower.Log(logger);

        final DriveTrajectoryFollower drivePID = driveControllerFactory.goodPIDF(PIDFlog);

        var m_shooter = new DrumShooter(logger, 3, 13, 27, 58, 100);

        SensorInterface m_sensors = new MockSensors();
        final FeederSubsystem feeder = new FeederSubsystem(logger, m_sensors);

        Intake intake = new Intake(logger, m_sensors);

        final TrajectoryVisualization viz = new TrajectoryVisualization(logger);

        AutoMaker m_AutoMaker = new AutoMaker(
                logger,
                drive,
                driveControllerFactory,
                drivePID,
                0,
                feeder,
                m_shooter,
                intake,
                m_sensors,
                swerveKinodynamics,
                viz);

        m_AutoMaker.fourNoteAuto(Alliance.Red, m_sensors);
        m_AutoMaker.fourNoteAuto(Alliance.Blue, m_sensors);
        m_AutoMaker.citrus(Alliance.Red);
        m_AutoMaker.citrus(Alliance.Blue);
        m_AutoMaker.citrusv2(Alliance.Red);
        m_AutoMaker.citrusv2(Alliance.Blue);
        m_AutoMaker.sibling(Alliance.Red);
        m_AutoMaker.sibling(Alliance.Blue);
        m_AutoMaker.complementAuto(Alliance.Red);
        m_AutoMaker.complementAuto(Alliance.Blue);

        m_AutoMaker.eightNoteAuto(Alliance.Red);
        m_AutoMaker.eightNoteAuto(Alliance.Blue);

    }

}
