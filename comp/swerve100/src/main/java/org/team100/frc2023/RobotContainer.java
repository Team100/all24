package org.team100.frc2023;

import java.io.IOException;

import org.team100.lib.commands.Defense;
import org.team100.lib.commands.DriveManually;
import org.team100.lib.commands.DriveWithHeading;
import org.team100.lib.commands.FancyTrajectory;
import org.team100.lib.commands.ManualMode;
import org.team100.lib.commands.ResetPose;
import org.team100.lib.commands.Rotate;
import org.team100.lib.commands.SetRotation;
import org.team100.lib.config.AllianceSelector;
import org.team100.lib.config.AutonSelector;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.hid.Control;
import org.team100.lib.hid.DualXboxControl;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.VisionDataProvider;
import org.team100.lib.motion.drivetrain.Heading;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SpeedLimitsFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.SwerveModuleCollectionFactory;
import org.team100.lib.motion.drivetrain.SwerveModuleCollectionInterface;
import org.team100.lib.motion.drivetrain.SwerveModuleFactory;
import org.team100.lib.motion.drivetrain.VeeringCorrection;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.motion.drivetrain.kinematics.SwerveDriveKinematicsFactory;
import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.sensors.RedundantGyroInterface;
import org.team100.lib.swerve.SwerveKinematicLimits;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.DrawCircle;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    public static class Config {

        //////////////////////////////////////
        // SHOW MODE
        //
        // Show mode is for younger drivers to drive the robot slowly.
        //
        // TODO: make a physical show mode switch.
        public boolean SHOW_MODE = true;
        //
        //////////////////////////////////////

        public double kDriveCurrentLimit = 30;
        // public double kDriveCurrentLimit = SHOW_MODE ? 20 : 60;
    }

    private final Config m_config = new Config();

    private final Telemetry t = Telemetry.get();

    private final AutonSelector m_autonSelector;
    private final AllianceSelector m_allianceSelector;

    private final Heading m_heading;
    private final LEDIndicator m_indicator;
    private final RedundantGyroInterface ahrsclass;
    private final Field2d m_field;
    private final AprilTagFieldLayoutWithCorrectOrientation layout;
    private final SwerveDriveSubsystem m_robotDrive;
    private final SwerveModuleCollectionInterface m_modules;
    private final SwerveDriveKinematics m_kinematics;
    private final Command m_auton;
    private final FrameTransform m_frameTransform;

    private final Control control;

    public RobotContainer() throws IOException {

        m_autonSelector = new AutonSelector();
        t.log(Level.INFO, "/Routine", getRoutine());

        m_allianceSelector = new AllianceSelector();
        t.log(Level.INFO, "/Alliance", m_allianceSelector.alliance().name());

        m_indicator = new LEDIndicator(8);

        Identity identity = Identity.get();
        // override the correct identity for testing.
        // Identity identity = Identity.COMP_BOT;

        ahrsclass = new RedundantGyro.Factory(identity).get();
        m_heading = new Heading(ahrsclass);
        m_field = new Field2d();

        SpeedLimits speedLimits = SpeedLimitsFactory.get(identity, m_config.SHOW_MODE);
        m_kinematics = SwerveDriveKinematicsFactory.get(identity);

        // TODO replace with SpeedLimits.
        SwerveKinematicLimits m_kinematicLimits = new SwerveKinematicLimits();
        // TODO: fix these limits
        m_kinematicLimits.kMaxDriveVelocity = 4;
        m_kinematicLimits.kMaxDriveAcceleration = 2;
        m_kinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(750.0);

        VeeringCorrection veering = new VeeringCorrection(m_heading::getHeadingRateNWU);

        m_frameTransform = new FrameTransform(veering);

        Experiments experiments = new Experiments(identity);

        SwerveModuleFactory moduleFactory = new SwerveModuleFactory(experiments, m_config.kDriveCurrentLimit);
        m_modules = new SwerveModuleCollectionFactory(identity, moduleFactory).get();

        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                new Pose2d(),
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.1, 0.1, 0.4));

        // TODO: make this override work better
        // if (m_allianceSelector.alliance() == DriverStation.Alliance.Blue) {
        layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout("2023-studies.json");
        // } else { // red
        // layout =
        // AprilTagFieldLayoutWithCorrectOrientation.redLayout("2023-studies.json");
        // }

        VisionDataProvider visionDataProvider = new VisionDataProvider(
                layout,
                poseEstimator,
                poseEstimator::getEstimatedPosition);
        visionDataProvider.enable();

        SwerveLocal swerveLocal = new SwerveLocal(
                experiments,
                speedLimits,
                m_kinematics,
                m_modules);

        m_robotDrive = new SwerveDriveSubsystem(
                m_heading,
                poseEstimator,
                m_frameTransform,
                swerveLocal,
                m_field);

        m_auton = new Defense(m_robotDrive);

        ////////////////////////////
        // DRIVETRAIN COMMANDS
        // TODO: control selection using names
        // control = new JoystickControl();

        control = new DualXboxControl();
        control.defense(new Defense(m_robotDrive));
        control.resetRotation0(new SetRotation(m_robotDrive, new Rotation2d(0)));
        control.resetRotation180(new SetRotation(m_robotDrive, Rotation2d.fromDegrees(180)));

        ManualMode manualMode = new ManualMode();
        SpeedLimits slow = new SpeedLimits(0.4, 1.0, 0.5, 1.0);
        control.driveSlow(new DriveManually(manualMode, control::twist, m_robotDrive, slow));
        SpeedLimits medium = new SpeedLimits(2.0, 2.0, 0.5, 1.0);
        control.driveMedium(new DriveManually(manualMode, control::twist, m_robotDrive, medium));
        // TODO: make the reset configurable
        // control.resetPose(new ResetPose(m_robotDrive, 0, 0, 0));
        control.resetPose(new ResetPose(m_robotDrive, 0, 0, Math.PI));
        control.rotate0(new Rotate(m_robotDrive, m_heading, speedLimits, new Timer(), 0));

        // new Circle(new Pose2d(1, 1, Rotation2d.fromDegrees(180))), m_robotDrive,
        // m_kinematics

        // Circle circle =

        // Pose2d[] goalArr = {
        // new Pose2d(-2.199237, -0.400119, Rotation2d.fromDegrees(180)),
        // new Pose2d(-2.199237, 1, Rotation2d.fromDegrees(180)),
        // new Pose2d(-3.312756, 1, Rotation2d.fromDegrees(180)),
        // new Pose2d(-3.312756, -0.400119, Rotation2d.fromDegrees(180)),
        // new Pose2d(-2.199237, -0.400119, Rotation2d.fromDegrees(180))

        // };

        // Pose2d[] goalArr = { new Pose2d(1, 1, Rotation2d.fromDegrees(180)),
        // new Pose2d(1, -1, Rotation2d.fromDegrees(180)),
        // new Pose2d(-1, -1, Rotation2d.fromDegrees(180)),
        // new Pose2d(-1, 1, Rotation2d.fromDegrees(180)),
        // new Pose2d(1, 1, Rotation2d.fromDegrees(180))

        // };

        Pose2d[] goalArr = { new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(180)),
                new Pose2d(0.5, -0.5, Rotation2d.fromDegrees(180)),
                new Pose2d(-0.5, -0.5, Rotation2d.fromDegrees(180)),
                new Pose2d(-0.5, 0.5, Rotation2d.fromDegrees(180)),
                new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(180))

        };
        // control.circle(new Circle(new Pose2d(-2, 0, Rotation2d.fromDegrees(180)),
        // m_robotDrive, m_kinematics));
        control.circle(new DrawCircle(goalArr, m_robotDrive, m_kinematics));

        control.driveWithFancyTrajec(new FancyTrajectory(m_kinematics, m_kinematicLimits, m_robotDrive));

        ///////////////////////////
        // DRIVE

        if (m_config.SHOW_MODE) {
            m_robotDrive.setDefaultCommand(
                    new DriveManually(
                            manualMode,
                            control::twist,
                            m_robotDrive,
                            speedLimits));
        } else {
            m_robotDrive.setDefaultCommand(
                    new DriveWithHeading(
                            control::twist,
                            m_robotDrive,
                            m_heading,
                            speedLimits,
                            new Timer(),
                            control::desiredRotation));
        }

    }

    public void scheduleAuton() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    public void cancelAuton() {
        if (m_auton == null)
            return;
        m_auton.cancel();
    }

    public void runTest2() {
        XboxController controller0 = new XboxController(0);
        boolean rearLeft = controller0.getAButton();
        boolean rearRight = controller0.getBButton();
        boolean frontLeft = controller0.getXButton();
        boolean frontRight = controller0.getYButton();
        double driveControl = controller0.getLeftY();
        double turnControl = controller0.getLeftX();
        double[][] desiredOutputs = {
                { frontLeft ? driveControl : 0, frontLeft ? turnControl : 0 },
                { frontRight ? driveControl : 0, frontRight ? turnControl : 0 },
                { rearLeft ? driveControl : 0, rearLeft ? turnControl : 0 },
                { rearRight ? driveControl : 0, rearRight ? turnControl : 0 }
        };
        m_robotDrive.test(desiredOutputs);
    }

    public double getRoutine() {
        return m_autonSelector.routine();
    }

    public void ledStart() {
        m_indicator.set(State.ORANGE);
    }

    public void ledStop() {
        m_indicator.close();
    }

    public void red() {
        m_indicator.set(State.RED);
    }

    public void green() {
        m_indicator.set(State.GREEN);
    }

    // this keeps the tests from conflicting via the use of simulated HAL ports.
    public void close() {
        m_autonSelector.close();
        m_allianceSelector.close();
        m_indicator.close();
        m_modules.close();
    }
}
