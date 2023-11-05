package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.GyroIntegrator;

public class DriveSubsystem extends Subsystem {
    private static final boolean USE_INTEGRATOR = true;
    private static final double ksVolts = 0.2;
    private static final double kvVoltSecondsPerMeter = 4;
    private static final double kaVoltSecondsSquaredPerMeter = 0.5;
    private static final double kvVoltSecondsPerRadian = 4;
    private static final double kaVoltSecondsSquaredPerRadian = 0.5;
    private static final double kTrackwidthMeters = 0.69;
    private static final double kWheelDiameterMeters = 0.15;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);
    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            ksVolts,
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter);

    private final MotorControllerGroup m_leftMotors;
    private final MotorControllerGroup m_rightMotors;
    private final DifferentialDrive m_drive;
    private final Encoder m_leftEncoder;
    private final Encoder m_rightEncoder;
    private final ADXRS450_Gyro m_gyro;
    private final GyroIntegrator m_gyroIntegrator;
    private final DifferentialDriveOdometry m_odometry;
    private final EncoderSim m_leftEncoderSim;
    private final EncoderSim m_rightEncoderSim;
    private final Field2d m_fieldSim;
    private final ADXRS450_GyroSim m_gyroSim;
    private final DifferentialDrivetrainSim m_drivetrainSimulator;

    public DriveSubsystem() {
        m_leftMotors = new MotorControllerGroup(new PWMSparkMax(0), new PWMSparkMax(1));
        m_rightMotors = new MotorControllerGroup(new PWMSparkMax(2), new PWMSparkMax(3));
        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_leftEncoder = new Encoder(0, 1, false);
        m_rightEncoder = new Encoder(2, 3, true);
        m_gyro = new ADXRS450_Gyro();
        m_gyroIntegrator = new GyroIntegrator();
        m_rightMotors.setInverted(true);
        final int kEncoderCPR = 1024;
        final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / kEncoderCPR;
        m_leftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(
                Rotation2d.fromDegrees(getHeading()),
                m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance());

        if (RobotBase.isSimulation()) {

            final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
                    kvVoltSecondsPerMeter,
                    kaVoltSecondsSquaredPerMeter,
                    kvVoltSecondsPerRadian,
                    kaVoltSecondsSquaredPerRadian);
            final DCMotor kDriveGearbox = DCMotor.getCIM(2);
            final double kDriveGearing = 8;
            m_drivetrainSimulator = new DifferentialDrivetrainSim(
                    kDrivetrainPlant,
                    kDriveGearbox,
                    kDriveGearing,
                    kTrackwidthMeters,
                    kWheelDiameterMeters / 2.0,
                    VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
            m_leftEncoderSim = new EncoderSim(m_leftEncoder);
            m_rightEncoderSim = new EncoderSim(m_rightEncoder);
            m_gyroSim = new ADXRS450_GyroSim(m_gyro);
            m_fieldSim = new Field2d();
            SmartDashboard.putData("Field", m_fieldSim);
        } else {
            m_drivetrainSimulator = null;
            m_leftEncoderSim = null;
            m_rightEncoderSim = null;
            m_gyroSim = null;
            m_fieldSim = null;
        }
    }

    @Override
    public void periodic() {
        m_odometry.update(
                Rotation2d.fromDegrees(getHeading()),
                m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance());
        m_gyroIntegrator.addRateNWU(-m_gyro.getRate(), 0.020);
        m_fieldSim.setRobotPose(getPose());
    }

    @Override
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the simulation,
        // and write the simulated positions and velocities to our simulated encoder and
        // gyro.
        // We negate the right side so that positive voltages make the right side
        // move forward.
        m_drivetrainSimulator.setInputs(
                m_leftMotors.get() * RobotController.getBatteryVoltage(),
                m_rightMotors.get() * RobotController.getBatteryVoltage());
        Rotation2d oldHeading = m_drivetrainSimulator.getHeading();
        m_drivetrainSimulator.update(0.020);
        Rotation2d newHeading = m_drivetrainSimulator.getHeading();
        Rotation2d diffHeading = newHeading.minus(oldHeading);
        double omega = diffHeading.getDegrees() / 0.020;

        m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
        m_gyroSim.setRate(-omega);
    }

    /**
     * Returns the current being drawn by the drivetrain. This works in SIMULATION
     * ONLY! If you want
     * it to work elsewhere, use the code in
     * {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
     *
     * @return The drawn current in Amps.
     */
    public double getDrawnCurrentAmps() {
        return m_drivetrainSimulator.getCurrentDrawAmps();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_drivetrainSimulator.setPose(pose);
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(getHeading()),
                m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance(),
                pose);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(rightVolts);
        m_drive.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
        m_gyroIntegrator.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        if (USE_INTEGRATOR)
            return m_gyroIntegrator.getHeadingNWU();
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * -1.0;
    }
}
