package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.encoder.drive.FalconDriveEncoder;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.drive.FalconDriveMotor;
import org.team100.lib.motor.turning.FalconTurningMotor;
import org.team100.lib.profile.ChoosableProfile;
import org.team100.lib.units.Angle;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class WCPSwerveModule100 extends SwerveModule100 {
    // WCP 4 inch wheel
    private static final double kWheelDiameterM = 0.1015;
    // see wcproducts.com, this is the "fast" ratio.
    private static final double kDriveReduction = 5.50;
    private static final double driveEncoderDistancePerTurn = kWheelDiameterM * Math.PI / kDriveReduction;

    public static WCPSwerveModule100 get(
            String name,
            double currentLimit,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics) {

        VelocityServo<Distance> driveServo = driveServo(
                name,
                currentLimit,
                driveMotorCanId);

        PositionServo<Angle> turningServo = turningServo(
                name,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                kinodynamics);

        return new WCPSwerveModule100(name, driveServo, turningServo);
    }

    private static VelocityServo<Distance> driveServo(
            String name,
            double currentLimit,
            int driveMotorCanId) {
        FalconDriveMotor driveMotor = new FalconDriveMotor(
                name,
                driveMotorCanId,
                currentLimit,
                kDriveReduction,
                kWheelDiameterM);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(
                name,
                driveMotor,
                driveEncoderDistancePerTurn);
        PIDController driveController = new PIDController( //
                0.1, // kP //1.2
                0, // kI //0.3
                0.0); // kD
        // Note very low windup limit.
        driveController.setIntegratorRange(-0.01, 0.01);
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward( //
                0.06, // kS
                0.3, // kV
                0.025); // kA
        return new VelocityServo<>(
                drive(name),
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);
    }

    private static PositionServo<Angle> turningServo(
            String name,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics) {
        final double turningGearRatio = 1.0;
        FalconTurningMotor turningMotor = new FalconTurningMotor(
                name,
                turningMotorCanId);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(
                name,
                turningEncoderChannel,
                turningOffset,
                turningGearRatio,
                AnalogTurningEncoder.Drive.DIRECT);
        PIDController angleVelocityController = new PIDController(
                2.86, // kP
                0, // kI
                0, // kD
                dt);
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward( //
                0.0006, // kS: Multiplied by around 20 of previous value as that is how much we changed
                        // P by 0.0005
                0.005, // kV: Since we are decreasing the value of how much the PID system does we need
                       // to conpensate for making feedforward larger as well
                0); // kA
        VelocityServo<Angle> turningVelocityServo = new VelocityServo<>(
                turning(name),
                turningMotor,
                turningEncoder,
                angleVelocityController,
                turningFeedforward);

        PIDController turningPositionController = new PIDController(
                2.86, // kP
                0.06, // kI
                0, // kD
                dt);
        turningPositionController.enableContinuousInput(-Math.PI, -Math.PI);
        turningPositionController.setTolerance(0.1, 0.1);

        ChoosableProfile profile = kinodynamics.getSteeringProfile();
        PositionServo<Angle> turningServo = new PositionServo<>(
                turning(name),
                turningVelocityServo,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                turningPositionController,
                profile,
                Angle.instance);
        turningServo.reset();
        return turningServo;
    }

    private WCPSwerveModule100(
            String name,
            VelocityServo<Distance> driveServo,
            PositionServo<Angle> turningServo) {
        super(name, driveServo, turningServo);
        //
    }
}
