package org.team100.lib.motion.drivetrain;

import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.encoder.drive.FalconDriveEncoder;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.drive.FalconDriveMotor;
import org.team100.lib.motor.turning.CANTurningMotor;
import org.team100.lib.motor.turning.FalconTurningMotor;
import org.team100.lib.motor.turning.PWMTurningMotor;
import org.team100.lib.profile.ChoosableProfile;
import org.team100.lib.units.Angle;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveModuleFactory {
    private final Experiments experiments;
    private final double currentLimit;

    public SwerveModuleFactory(Experiments experiments, double currentLimit) {
        this.experiments = experiments;
        this.currentLimit = currentLimit;
    }

    /**
     * 
     * @param name                  may not contain slashes
     * @param driveMotorCanId
     * @param turningMotorCanId
     * @param turningEncoderChannel
     * @param turningOffset
     * @return
     */
    public SwerveModule100 WCPModule(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        final double kWheelDiameterMeters = 0.1015; // WCP 4 inch wheel
        // TODO: verify the drive ratio
        final double kDriveReduction = 5.50; // see wcproducts.com, this is the "fast" ratio.
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0;

        FalconDriveMotor driveMotor = new FalconDriveMotor(
                name,
                driveMotorCanId,
                currentLimit,
                kDriveReduction,
                kWheelDiameterMeters);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(
                name,
                driveMotor,
                driveEncoderDistancePerTurn);

        FalconTurningMotor turningMotor = new FalconTurningMotor(name, turningMotorCanId);

        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(
                name, turningEncoderChannel,
                turningOffset,
                turningGearRatio, AnalogTurningEncoder.Drive.DIRECT);

        // DRIVE PID
        PIDController driveController = new PIDController( //
                0.1, // kP //1.2
                0, // kI: nonzero I eliminates small errors, e.g. to finish rotations. //0.3
                0.0); // kD
        driveController.setIntegratorRange(-0.01, 0.01); // Note very low windup limit.

        // TODO: shorter period
        PIDController turningController2 = new PIDController(2.86, 0.06, 0, 0.02);
        turningController2.enableContinuousInput(-Math.PI, -Math.PI);
        turningController2.setTolerance(0.1, 0.1);

        // DRIVE FF
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward( //
                0.06, // kS
                0.3, // kV
                0.025); // kA

        // TURNING FF
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward( //
                0.0006, // kS: Multiplied by around 20 of previous value as that is how much we changed
                        // P by 0.0005
                0.005, // kV: Since we are decreasing the value of how much the PID system does we need
                       // to conpensate for making feedforward larger as well
                0); // kA: I have no idea what this value should be

        VelocityServo<Distance> driveServo = new VelocityServo<>(
                experiments,
                drive(name),
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);

        // TODO: tune this
        PIDController angleVelocityController = new PIDController(2.86, 0, 0, 0.02);
        VelocityServo<Angle> turningVelocityServo = new VelocityServo<>(
                experiments,
                turning(name),
                turningMotor,
                turningEncoder,
                angleVelocityController,
                turningFeedforward);

        ChoosableProfile profile = new ChoosableProfile(
                20 * Math.PI, // max angular speed radians/sec
                20 * Math.PI, // max accel radians/sec/sec
                ChoosableProfile.Mode.TRAPEZOID);
        PositionServo<Angle> turningServo = new PositionServo<>(
                turning(name),
                turningVelocityServo,
                turningEncoder,
                20 * Math.PI,
                turningController2,
                profile,
                Angle.instance);
        turningServo.reset();
        return new SwerveModule100(name, driveServo, turningServo);
    }

    /**
     * For outboard closed-loop control.
     */
    public SwerveModule100 AMCANModule(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            AnalogTurningEncoder.Drive turningDrive) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        final double kWheelDiameterMeters = 0.1016; // AndyMark Swerve & Steer has 4 inch wheel
        // TODO: verify the wheel diameter
        final double kDriveReduction = 6.67 * 9 / 10; // see andymark.com/products/swerve-and-steer
        // TODO Temperarely added a modifyer to make this more realistic through some
        // testing, we will need to make this a real value
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0; // andymark ma3 encoder is 1:1

        FalconDriveMotor driveMotor = new FalconDriveMotor(
                name,
                driveMotorCanId,
                currentLimit,
                kDriveReduction,
                kWheelDiameterMeters);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name,
                driveMotor,
                driveEncoderDistancePerTurn);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(
                name,
                turningEncoderChannel,
                turningOffset,
                turningGearRatio, turningDrive);

        CANTurningMotor turningMotor = new CANTurningMotor(name, turningMotorCanId, turningEncoder);

        // DRIVE PID
        PIDController driveController = new PIDController( //
                0.1, // kP
                0, // kI
                0); // kD

        // TODO: shorter period
        PIDController turningController2 = new PIDController(5, 0, 0, 0.02);
        turningController2.enableContinuousInput(-Math.PI, Math.PI);
        turningController2.setTolerance(0.1, 0.1);

        // DRIVE FF
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward( //
                0.0, // kS
                .5); // kV

        // TURNING FF
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward( //
                .25, // kS
                0.015); // kV

        // TODO: what is this?
        // SimpleMotorFeedforward headingDriveFeedForward = new SimpleMotorFeedforward(
        // //
        // 0.05, // kS: friction is unimportant
        // 0.35, // kV: from experiment; higher than AM modules, less reduction gear
        // 0.08); // kA: I have no idea what this value should be

        VelocityServo<Distance> driveServo = new VelocityServo<>(
                experiments,
                drive(name),
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);

        // TODO: tune this
        PIDController angleVelocityController = new PIDController(5, 0, 0, 0.02);
        VelocityServo<Angle> turningVelocityServo = new VelocityServo<>(
                experiments,
                turning(name),
                turningMotor,
                turningEncoder,
                angleVelocityController,
                turningFeedforward);

        // TURNING PID

        ChoosableProfile profile = new ChoosableProfile(
                20 * Math.PI, // speed rad/s
                20 * Math.PI, // accel rad/s/sturningConstraints,
                ChoosableProfile.Mode.TRAPEZOID);
        PositionServo<Angle> turningServo = new PositionServo<>(
                turning(name),
                turningVelocityServo,
                turningEncoder,
                20 * Math.PI, // vel
                turningController2,
                profile,
                Angle.instance);
        turningServo.reset();
        return new SwerveModule100(name, driveServo, turningServo);

    }

    /**
     * Stock AndyMark module with Falcon drive and PWM 775 steering.
     * 
     * @param name may not contain slashes
     */
    public SwerveModule100 AMModule(
            String name,
            int driveMotorCanId,
            int turningMotorChannel,
            int turningEncoderChannel,
            double turningOffset) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        final double kWheelDiameterMeters = 0.09628; // AndyMark Swerve & Steer has 4 inch wheel
        final double kDriveReduction = 6.67; // see andymark.com/products/swerve-and-steer
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0; // andymark ma3 encoder is 1:1
        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit, kDriveReduction,
                kWheelDiameterMeters);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);
        PWMTurningMotor turningMotor = new PWMTurningMotor(name, turningMotorChannel);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(
                name,
                turningEncoderChannel,
                turningOffset,
                turningGearRatio,
                AnalogTurningEncoder.Drive.DIRECT);

        // DRIVE PID
        PIDController driveController = new PIDController(//
                0.1, // kP
                0, // kI
                0);// kD

        // TODO: shorter period
        PIDController turningController2 = new PIDController(0.5, 0, 0, 0.02);
        turningController2.enableContinuousInput(-Math.PI, Math.PI);
        turningController2.setTolerance(0.1, 0.1);

        // DRIVE FF
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(//
                0.04, // kS makes it go further when almost at goal
                0.23, // kV
                0.02); // kA

        // TURNING FF
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(//
                0.05, // kS
                0.003, // kV
                0); // kA

        VelocityServo<Distance> driveServo = new VelocityServo<>(
                experiments,
                drive(name),
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);

        // TODO: tune this
        PIDController angleVelocityController = new PIDController(0.5, 0, 0, 0.02);
        VelocityServo<Angle> turningVelocityServo = new VelocityServo<>(
                experiments,
                turning(name),
                turningMotor,
                turningEncoder,
                angleVelocityController,
                turningFeedforward);

        ChoosableProfile profile = new ChoosableProfile(
                20 * Math.PI,
                20 * Math.PI,
                ChoosableProfile.Mode.TRAPEZOID);
        PositionServo<Angle> turningServo = new PositionServo<>(
                turning(name),
                turningVelocityServo,
                turningEncoder,
                20 * Math.PI,
                turningController2,
                profile,
                Angle.instance);
        turningServo.reset();
        return new SwerveModule100(name, driveServo, turningServo);
    }

    /**
     * @param name may not contain slashes
     * @return
     */
    public SwerveModule100 SimulatedModule(String name) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        SimulatedMotor<Distance> driveMotor = new SimulatedMotor<>(drive(name));
        // TODO: what should the reduction be here? is the drive motor velocity
        // command actually the velocity after reduction?
        SimulatedEncoder<Distance> driveEncoder = new SimulatedEncoder<>(
                Distance.instance,
                drive(name),
                driveMotor,
                1);
        PIDController driveController = new PIDController(//
                0.1, // kP
                0, // kI
                0);// kD

        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(//
                0.04, // kS makes it go further when almost at goal
                0.23, // kV
                0.02); // kA

        VelocityServo<Distance> driveServo = new VelocityServo<>(
                experiments,
                drive(name),
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);

        SimulatedMotor<Angle> turningMotor = new SimulatedMotor<>(turning(name));
        // TODO: what should the reduction be here? is the turning motor velocity
        // command actually the velocity after reduction?
        SimulatedEncoder<Angle> turningEncoder = new SimulatedEncoder<>(
                Angle.instance, 
                turning(name),
                 turningMotor,
                  1);
        PIDController angleVelocityController = new PIDController(0.5, 0, 0, 0.02);
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(//
                0.05, // kS
                0.003, // kV
                0); // kA
        VelocityServo<Angle> turningVelocityServo = new VelocityServo<>(
                experiments,
                turning(name),
                turningMotor,
                turningEncoder,
                angleVelocityController,
                turningFeedforward);

        // NOTE: these PID values are untuned
        // NOTE high P value, it's in rad/s not [-1,1]
        PIDController turningController2 = new PIDController(20, 0, 0, 0.02);
        turningController2.enableContinuousInput(-Math.PI, Math.PI);
        // note low tolerance
        turningController2.setTolerance(0.05, 0.05);

        ChoosableProfile profile = new ChoosableProfile(
                20 * Math.PI,
                20 * Math.PI,
                ChoosableProfile.Mode.TRAPEZOID);
        PositionServo<Angle> turningServo = new PositionServo<>(
                turning(name),
                turningVelocityServo,
                turningEncoder,
                20 * Math.PI,
                turningController2,
                profile,
                Angle.instance);
        turningServo.reset();
        return new SwerveModule100(name, driveServo, turningServo);
    }

    private String turning(String name) {
        return name + "/Turning";
    }

    private String drive(String name) {
        return name + "/Drive";
    }

}
