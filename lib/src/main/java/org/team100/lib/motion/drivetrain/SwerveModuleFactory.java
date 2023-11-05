package org.team100.lib.motion.drivetrain;

import org.team100.lib.encoder.drive.FalconDriveEncoder;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.drive.FalconDriveMotor;
import org.team100.lib.motor.turning.CANTurningMotor;
import org.team100.lib.motor.turning.FalconTurningMotor;
import org.team100.lib.motor.turning.PWMTurningMotor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModuleFactory {
    private final Experiments experiments;
    private final double currentLimit;

    public SwerveModuleFactory(Experiments experiments, double currentLimit) {
        this.experiments = experiments;
        this.currentLimit = currentLimit;
    }

    public SwerveModule WCPModule(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset) {
        final double kWheelDiameterMeters = 0.1015; // WCP 4 inch wheel
        final double kDriveReduction = 5.50; // see wcproducts.com, this is the "fast" ratio. 
        //TODO Temperarely added a modifyer to make this more realistic through some testing, we will need to make this a real value
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0;

        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit, kDriveReduction, kWheelDiameterMeters);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);

        FalconTurningMotor turningMotor = new FalconTurningMotor(name, turningMotorCanId);

        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
                turningGearRatio);

        // DRIVE PID
        PIDController driveController = new PIDController( //
                0.1, // kP //1.2
                0, // kI: nonzero I eliminates small errors, e.g. to finish rotations. //0.3
                0.0); // kD
        driveController.setIntegratorRange(-0.01, 0.01); // Note very low windup limit.

        // TURNING PID
        ProfiledPIDController turningController = new ProfiledPIDController(
                1, // kP: High P to keep the measurments acurate while maintaining an agresive wheel turning
                0, // kI
                0, // kD
                new TrapezoidProfile.Constraints( //
                        20 * Math.PI, // max angular speed radians/sec
                        20 * Math.PI)); // max accel radians/sec/sec
        turningController.enableContinuousInput(0, 2 * Math.PI);

        // DRIVE FF
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward( //
                0.06, // kS
                0.3, // kV
                0.025); // kA

        // TURNING FF
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward( //
                0.0005, // kS: Multiplied by around 20 of previous value as that is how much we changed P by 
                0.005, // kV: Since we are decreasing the value of how much the PID system does we need to conpensate for making feedforward larger as well
                0); // kA: I have no idea what this value should be

        DriveServo driveServo = new DriveServo(
                experiments,
                name,
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);
        TurningServo turningServo = new TurningServo(
                experiments,
                name,
                turningMotor,
                turningEncoder,
                turningController,
                turningFeedforward);

        return new SwerveModule(driveServo, turningServo);
    }

    // for 8048's config and new Offloaded PID
    public SwerveModule Swerve2CAN(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset) {
        final double kWheelDiameterMeters = 0.1016; // AndyMark Swerve & Steer has 4 inch wheel
        final double kDriveReduction = 6.67*9/10; // see andymark.com/products/swerve-and-steer 
        //TODO Temperarely added a modifyer to make this more realistic through some testing, we will need to make this a real value
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0; // andymark ma3 encoder is 1:1

        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit, kDriveReduction, kWheelDiameterMeters);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
                turningGearRatio);
        CANTurningMotor turningMotor = new CANTurningMotor(name, turningMotorCanId, turningEncoder, 2);

        // DRIVE PID
        PIDController driveController = new PIDController( //
                0.1, // kP
                0, // kI
                0); // kD

        // TURNING PID
        ProfiledPIDController turningController = new ProfiledPIDController( //
                5, // kP
                0, // kI
                0, // kD
                new TrapezoidProfile.Constraints(
                        20 * Math.PI, // speed rad/s
                        20 * Math.PI)); // accel rad/s/s
        turningController.enableContinuousInput(0, 2 * Math.PI);

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

        DriveServo driveServo = new DriveServo(
                experiments,
                name,
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);
        TurningServo turningServo = new TurningServo(
                experiments,
                name,
                turningMotor,
                turningEncoder,
                turningController,
                turningFeedforward);

        return new SwerveModule(driveServo, turningServo);

    }

    public SwerveModule Swerve1CAN(
        String name,
        int driveMotorCanId,
        int turningMotorCanId,
        int turningEncoderChannel,
        double turningOffset) {
    final double kWheelDiameterMeters = 0.1016; // AndyMark Swerve & Steer has 4 inch wheel
    final double kDriveReduction = 6.67*9/10; // see andymark.com/products/swerve-and-steer 
    //TODO Temperarely added a modifyer to make this more realistic through some testing, we will need to make this a real value
    final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
    final double turningGearRatio = 1.0; // andymark ma3 encoder is 1:1

    FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit, kDriveReduction, kWheelDiameterMeters);
    FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);
    AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
            turningGearRatio);
    CANTurningMotor turningMotor = new CANTurningMotor(name, turningMotorCanId, turningEncoder, 1);

    // DRIVE PID
    PIDController driveController = new PIDController( //
            0.1, // kP
            0, // kI
            0); // kD

    // TURNING PID
    ProfiledPIDController turningController = new ProfiledPIDController( //
            5, // kP
            0, // kI
            0, // kD
            new TrapezoidProfile.Constraints(
                    20 * Math.PI, // speed rad/s
                    20 * Math.PI)); // accel rad/s/s
    turningController.enableContinuousInput(0, 2 * Math.PI);

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

    DriveServo driveServo = new DriveServo(
            experiments,
            name,
            driveMotor,
            driveEncoder,
            driveController,
            driveFeedforward);
    TurningServo turningServo = new TurningServo(
            experiments,
            name,
            turningMotor,
            turningEncoder,
            turningController,
            turningFeedforward);

    return new SwerveModule(driveServo, turningServo);
    }

    public SwerveModule AMModule(
            String name,
            int driveMotorCanId,
            int turningMotorChannel,
            int turningEncoderChannel,
            double turningOffset) {
        final double kWheelDiameterMeters = 0.09628; // AndyMark Swerve & Steer has 4 inch wheel
        final double kDriveReduction = 6.67; // see andymark.com/products/swerve-and-steer
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0; // andymark ma3 encoder is 1:1
        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit, kDriveReduction, kWheelDiameterMeters);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);
        PWMTurningMotor turningMotor = new PWMTurningMotor(name, turningMotorChannel);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
                turningGearRatio);

        // DRIVE PID
        PIDController driveController = new PIDController(//
                0.1, // kP
                0, // kI
                0);// kD

        // TURNING PID
        ProfiledPIDController turningController = new ProfiledPIDController(//
                0.5, // kP
                0, // kI
                0, // kD
                new TrapezoidProfile.Constraints(
                        20 * Math.PI, // speed rad/s
                        20 * Math.PI)); // accel rad/s/s
        turningController.enableContinuousInput(0, 2 * Math.PI);

        // Drive(IVE FF
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(//
                0.04, // kS makes it go further when almost at goal
                0.23, // kV
                0.02); // kA

        // TURNING FF
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(//
                0.05, // kS
                0.003, // kV
                0); // kA

        DriveServo driveServo = new DriveServo(
                experiments,
                name,
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);
        TurningServo turningServo = new TurningServo(
                experiments,
                name,
                turningMotor,
                turningEncoder,
                turningController,
                turningFeedforward);

        return new SwerveModule(driveServo, turningServo);
    }
}
