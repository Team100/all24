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

    public SwerveModule100 WCPModule(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset) {
        final double kWheelDiameterMeters = 0.1015; // WCP 4 inch wheel
        // TODO: verify the drive ratio
        final double kDriveReduction = 5.50; // see wcproducts.com, this is the "fast" ratio.
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0;

        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit, kDriveReduction,
                kWheelDiameterMeters);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);

        FalconTurningMotor turningMotor = new FalconTurningMotor(name, turningMotorCanId);

        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel,
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
        turningController2.setTolerance(0.1);

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
                name + "/drive",
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);

        // TODO: tune this
        PIDController angleVelocityController = new PIDController(2.86, 0, 0, 0.02);
        VelocityServo<Angle> turningVelocityServo = new VelocityServo<>(
                experiments,
                name + "/turn",
                turningMotor,
                turningEncoder,
                angleVelocityController,
                turningFeedforward);

        ChoosableProfile profile = new ChoosableProfile(
                20 * Math.PI, // max angular speed radians/sec
                20 * Math.PI, // max accel radians/sec/sec
                0,
                ChoosableProfile.Mode.TRAPEZOID);
        PositionServo<Angle> turningServo = new PositionServo<>(
                name,
                turningVelocityServo,
                turningEncoder,
                20 * Math.PI,
                turningController2,
                profile,
                MathUtil::angleModulus);

        return new SwerveModule100(driveServo, turningServo);
    }

    // for 8048's config and new Offloaded PID
    public SwerveModule100 AMCANModule(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            AnalogTurningEncoder.Drive turningDrive) {
        final double kWheelDiameterMeters = 0.1016; // AndyMark Swerve & Steer has 4 inch wheel
        // TODO: verify the wheel diameter
        final double kDriveReduction = 6.67 * 9 / 10; // see andymark.com/products/swerve-and-steer
        // TODO Temperarely added a modifyer to make this more realistic through some
        // testing, we will need to make this a real value
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0; // andymark ma3 encoder is 1:1

        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit, kDriveReduction,
                kWheelDiameterMeters);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel,
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
        turningController2.setTolerance(0.1);


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
                name + "/drive",
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);

        // TODO: tune this
        PIDController angleVelocityController = new PIDController(5, 0, 0, 0.02);
        VelocityServo<Angle> turningVelocityServo = new VelocityServo<>(
                experiments,
                name + "/turn",
                turningMotor,
                turningEncoder,
                angleVelocityController,
                turningFeedforward);

        // TURNING PID

        ChoosableProfile profile = new ChoosableProfile(
                20 * Math.PI, // speed rad/s
                20 * Math.PI, // accel rad/s/sturningConstraints,
                0, // jerk
                ChoosableProfile.Mode.TRAPEZOID);
        PositionServo<Angle> turningServo = new PositionServo<>(
                name,
                turningVelocityServo,
                turningEncoder,
                20 * Math.PI, // vel
                turningController2,
                profile,
                MathUtil::angleModulus);

        return new SwerveModule100(driveServo, turningServo);

    }

    public SwerveModule100 AMModule(
            String name,
            int driveMotorCanId,
            int turningMotorChannel,
            int turningEncoderChannel,
            double turningOffset) {
        final double kWheelDiameterMeters = 0.09628; // AndyMark Swerve & Steer has 4 inch wheel
        final double kDriveReduction = 6.67; // see andymark.com/products/swerve-and-steer
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0; // andymark ma3 encoder is 1:1
        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit, kDriveReduction,
                kWheelDiameterMeters);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);
        PWMTurningMotor turningMotor = new PWMTurningMotor(name, turningMotorChannel);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel,
                turningOffset,
                turningGearRatio, AnalogTurningEncoder.Drive.DIRECT);

        // DRIVE PID
        PIDController driveController = new PIDController(//
                0.1, // kP
                0, // kI
                0);// kD

        // TODO: shorter period
        PIDController turningController2 = new PIDController(0.5, 0, 0, 0.02);
        turningController2.enableContinuousInput(-Math.PI, Math.PI);
        turningController2.setTolerance(0.1);

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
                name + "/drive",
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);

        // TODO: tune this
        PIDController angleVelocityController = new PIDController(0.5, 0, 0, 0.02);
        VelocityServo<Angle> turningVelocityServo = new VelocityServo<>(
                experiments,
                name + "/turn",
                turningMotor,
                turningEncoder,
                angleVelocityController,
                turningFeedforward);

        ChoosableProfile profile = new ChoosableProfile(
                20 * Math.PI,
                20 * Math.PI,
                0, ChoosableProfile.Mode.TRAPEZOID);
        PositionServo<Angle> turningServo = new PositionServo<>(
                name,
                turningVelocityServo,
                turningEncoder,
                20 * Math.PI,
                turningController2,
                profile,
                MathUtil::angleModulus);

        return new SwerveModule100(driveServo, turningServo);
    }

    public SwerveModule100 SimulatedModule(String name) {
        SimulatedMotor<Distance> driveMotor = new SimulatedMotor<>(name + "/drive");
        SimulatedEncoder<Distance> driveEncoder = new SimulatedEncoder<>(name + "/drive", driveMotor);
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
                name + "/drive",
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);

        SimulatedMotor<Angle> turningMotor = new SimulatedMotor<>(name + "/turning");
        SimulatedEncoder<Angle> turningEncoder = new SimulatedEncoder<>(name + "/turning", turningMotor);
        PIDController angleVelocityController = new PIDController(0.5, 0, 0, 0.02);
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(//
                0.05, // kS
                0.003, // kV
                0); // kA
        VelocityServo<Angle> turningVelocityServo = new VelocityServo<>(
                experiments,
                name + "/turn",
                turningMotor,
                turningEncoder,
                angleVelocityController,
                turningFeedforward);

        // NOTE: these PID values are untuned
        PIDController turningController2 = new PIDController(5, 0, 0, 0.02);
        turningController2.enableContinuousInput(-Math.PI, Math.PI);
        turningController2.setTolerance(0.1);

        ChoosableProfile profile = new ChoosableProfile(
                20 * Math.PI,
                20 * Math.PI,
                0, ChoosableProfile.Mode.TRAPEZOID);
        PositionServo<Angle> turningServo = new PositionServo<>(
                name,
                turningVelocityServo,
                turningEncoder,
                20 * Math.PI,
                turningController2,
                profile,
                MathUtil::angleModulus);

        return new SwerveModule100(driveServo, turningServo);
    }

}
