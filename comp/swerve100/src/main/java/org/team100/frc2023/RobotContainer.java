package org.team100.frc2023;

import java.io.IOException;

import org.team100.lib.commands.arm.Sequence;
import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.DriveWithHeading;
import org.team100.lib.commands.drivetrain.FancyTrajectory;
import org.team100.lib.commands.drivetrain.ManualMode;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.Rotate;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.config.AllianceSelector;
import org.team100.lib.config.AutonSelector;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.ControlFactory;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.VisionDataProvider;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;
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
import org.team100.lib.selftest.Testable;
import org.team100.lib.sensors.Heading;
import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.sensors.RedundantGyroInterface;
import org.team100.lib.swerve.SwerveKinematicLimits;
import org.team100.lib.telemetry.Annunciator;
import org.team100.lib.telemetry.Monitor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.DrawCircle;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer implements Testable {
    public static class Config {

        //////////////////////////////////////
        // SHOW MODE
        //
        // Show mode is for younger drivers to drive the robot slowly.
        //
        // TODO: make a physical show mode switch.
        // TODO: make way more noticable.
        public boolean SHOW_MODE = false;
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
    private final SwerveDriveSubsystem m_drive;
    private final SwerveModuleCollectionInterface m_modules;
    private final SwerveDriveKinematics m_kinematics;
    private final Command m_auton;
    private final FrameTransform m_frameTransform;

    private final DriverControl control;
    private final DrawCircle m_drawCircle;
    private final Monitor m_monitor;

    // Identity-specific fields
    private final ArmSubsystem m_armSubsystem;
    private final ArmKinematics m_armKinematicsM;

    public RobotContainer(TimedRobot robot) throws IOException {

        m_autonSelector = new AutonSelector();
        t.log(Level.INFO, "/Routine", getRoutine());

        m_allianceSelector = new AllianceSelector();
        t.log(Level.INFO, "/Alliance", m_allianceSelector.alliance().name());

        m_indicator = new LEDIndicator(8);

        m_monitor = new Monitor(new Annunciator(0));
        robot.addPeriodic(m_monitor::periodic, 0.02);

        Identity identity = Identity.get();
        // override the correct identity for testing.
        // Identity identity = Identity.COMP_BOT;

        ahrsclass = new RedundantGyro.Factory(identity).get();
        m_heading = new Heading(ahrsclass);
        m_field = new Field2d();

        SpeedLimits speedLimits = SpeedLimitsFactory.get(identity, m_config.SHOW_MODE);
        m_kinematics = SwerveDriveKinematicsFactory.get(identity);

        // TODO replace with SpeedLimits.
        // TODO: fix these limits
        SwerveKinematicLimits m_kinematicLimits = new SwerveKinematicLimits(4, 2, 13);

        VeeringCorrection veering = new VeeringCorrection(m_heading::getHeadingRateNWU);

        m_frameTransform = new FrameTransform(veering);

        Experiments experiments = new Experiments(identity);

        SwerveModuleFactory moduleFactory = new SwerveModuleFactory(experiments, m_config.kDriveCurrentLimit);
        m_modules = new SwerveModuleCollectionFactory(identity, moduleFactory).get();

        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
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

        m_drive = new SwerveDriveSubsystem(
                m_heading,
                poseEstimator,
                m_frameTransform,
                swerveLocal,
                m_field);

        ////////////////////////////
        //
        // DRIVETRAIN COMMANDS
        //

        // control = new JoystickControl();
        // control = new DriverXboxControl();

        // selects the correct control class for whatever is plugged in
        control = new ControlFactory().getDriverControl();

        control.defense().whileTrue(m_drive.runInit(m_drive::defense));
        control.steer0().whileTrue(m_drive.runInit(m_drive::steer0));
        control.steer90().whileTrue(m_drive.runInit(m_drive::steer90));

        control.resetRotation0().onTrue(new SetRotation(m_drive, GeometryUtil.kRotationZero));
        control.resetRotation180().onTrue(new SetRotation(m_drive, Rotation2d.fromDegrees(180)));

        ManualMode manualMode = new ManualMode();
        SpeedLimits slow = new SpeedLimits(0.4, 1.0, 0.5, 1.0);
        control.driveSlow().whileTrue(new DriveManually(manualMode, control::twist, m_drive, slow));
        SpeedLimits medium = new SpeedLimits(2.0, 2.0, 0.5, 1.0);
        control.driveMedium().whileTrue(new DriveManually(manualMode, control::twist, m_drive, medium));

        // TODO: make the reset configurable
        // control.resetPose(new ResetPose(m_robotDrive, 0, 0, 0));
        control.resetPose().onTrue(new ResetPose(m_drive, 0, 0, Math.PI));
        control.rotate0().whileTrue(new Rotate(m_drive, m_heading, speedLimits, new Timer(), 0));

        m_drawCircle = new DrawCircle(experiments, m_drive, m_kinematics);
        control.circle().whileTrue(m_drawCircle);

        control.driveWithFancyTrajec().whileTrue(new FancyTrajectory(m_kinematics, m_kinematicLimits, m_drive));

        ///////////////////////////
        //
        // DRIVE
        //
        if (m_config.SHOW_MODE) {
            m_drive.setDefaultCommand(
                    new DriveManually(
                            manualMode,
                            control::twist,
                            m_drive,
                            speedLimits));
        } else {
            m_drive.setDefaultCommand(
                    new DriveWithHeading(
                            control::twist,
                            m_drive,
                            m_heading,
                            speedLimits,
                            new Timer(),
                            control::desiredRotation));
        }

        /////////////////////////////////
        //
        // IDENTITY-SPECIFIC PARTS
        //

        switch (identity) {
            case TEST_BOARD_6B:
                // TODO: use the correct identity.
                m_armSubsystem = new ArmSubsystem();
                m_armKinematicsM = new ArmKinematics(0.93, 0.92);
                m_auton = new Sequence(m_armSubsystem, m_armKinematicsM);
                break;
            default:
                m_armSubsystem = null;
                m_armKinematicsM = null;
                m_auton = m_drive.runInit(m_drive::defense);
                break;
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

    //////////////////////////////////
    //
    // for testing

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return m_drive;
    }

    public Command getDrawCircle() {
        return m_drawCircle;
    }

    public Monitor getMonitor() {
        return m_monitor;
    }
}
