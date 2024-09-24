package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.AnalogTurningEncoder;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.DutyCycleRotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.Profile100;

import edu.wpi.first.math.controller.PIDController;

public class WCPSwerveModule100 extends SwerveModule100 {
    private static final boolean USE_OUTBOARD_STEERING = true;
    private static final double kSteeringSupplyLimit = 10;
    private static final double kSteeringStatorLimit = 20;
    /**
     * WCP calls this "rotation ratio" here, we use the "flipped belt" which is the
     * fastest steering ratio.
     * https://docs.wcproducts.com/wcp-swervex/misc/other-configurations/ratio-options
     */
    private static final double kSteeringRatio = 10.29;

    /**
     * Flipped belt ratios.
     * 
     * See
     * https://docs.wcproducts.com/wcp-swervex/misc/other-configurations/ratio-options
     */
    public enum DriveRatio {
        FAST(5.5),
        MEDIUM(6.55);

        private double m_ratio;

        DriveRatio(double ratio) {
            m_ratio = ratio;
        }
    }

    // WCP 4 inch wheel
    private static final double kWheelDiameterM = 0.0975; // 0.1015

    public static WCPSwerveModule100 get(
            SupplierLogger2 parent,
            double supplyLimitAmps,
            double statorLimitAmps,
            int driveMotorCanId,
            DriveRatio ratio,
            Class<? extends RotaryPositionSensor> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            MotorPhase motorPhase) {

        // TODO: revisit these constants
        PIDConstants drivePidConstants = new PIDConstants(.2); // .2
        PIDConstants turningPidConstants = new PIDConstants(1.5); // 5
        Feedforward100 turningFF = Feedforward100.makeWCPSwerveTurningFalcon6();
        Feedforward100 driveFF = Feedforward100.makeWCPSwerveDriveFalcon6();

        LinearVelocityServo driveServo = driveServo(
                parent.child("Drive"),
                supplyLimitAmps,
                statorLimitAmps,
                driveMotorCanId,
                ratio,
                drivePidConstants,
                driveFF);

        AngularPositionServo turningServo = turningServo(
                parent.child("Turning"),
                encoderClass,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                kSteeringRatio,
                kinodynamics,
                drive,
                motorPhase,
                turningPidConstants,
                turningFF);

        return new WCPSwerveModule100(driveServo, turningServo);
    }

    private static LinearVelocityServo driveServo(
            SupplierLogger2 parent,
            double supplyLimit,
            double statorLimit,
            int driveMotorCanId,
            DriveRatio ratio,
            PIDConstants pidConstants,
            Feedforward100 ff) {
        Kraken6Motor driveMotor = new Kraken6Motor(
                parent,
                driveMotorCanId,
                MotorPhase.FORWARD,
                supplyLimit,
                statorLimit,
                pidConstants,
                ff);
        LinearMechanism mech = new SimpleLinearMechanism(
                driveMotor,
                new Talon6Encoder(parent, driveMotor),
                ratio.m_ratio,
                kWheelDiameterM);
        return new OutboardLinearVelocityServo(
                parent,
                mech);
    }

    private static AngularPositionServo turningServo(
            SupplierLogger2 parent,
            Class<? extends RotaryPositionSensor> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            double gearRatio,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            MotorPhase motorPhase,
            PIDConstants lowLevelPID,
            Feedforward100 ff) {
        Falcon6Motor turningMotor = new Falcon6Motor(
                parent,
                turningMotorCanId,
                motorPhase,
                kSteeringSupplyLimit,
                kSteeringStatorLimit,
                lowLevelPID,
                ff);
        RotaryPositionSensor turningEncoder = turningEncoder(
                encoderClass,
                parent,
                turningEncoderChannel,
                turningOffset,
                drive);

        Profile100 profile = kinodynamics.getSteeringProfile();

        AngularPositionServo turningServo = getTurningServo(
                parent,
                kinodynamics,
                turningMotor,
                turningEncoder,
                gearRatio,
                profile);

        turningServo.reset();
        return turningServo;
    }

    private static AngularPositionServo getTurningServo(
            SupplierLogger2 parent,
            SwerveKinodynamics kinodynamics,
            Falcon6Motor turningMotor,
            RotaryPositionSensor turningEncoder,
            double turningGearRatio,
            Profile100 profile) {

        Talon6Encoder builtInEncoder = new Talon6Encoder(
                parent,
                turningMotor);

        RotaryMechanism mech = new RotaryMechanism(
                parent,
                turningMotor,
                builtInEncoder,
                turningGearRatio);

        if (USE_OUTBOARD_STEERING)
            return getOutboard(
                    parent,
                    turningEncoder,
                    profile,
                    mech);

        return getOnboard(
                parent,
                kinodynamics,
                turningEncoder,
                profile,
                mech);
    }

    private static AngularPositionServo getOutboard(
            SupplierLogger2 parent,
            RotaryPositionSensor turningEncoder,
            Profile100 profile,
            RotaryMechanism mech) {
        CombinedEncoder combinedEncoder = new CombinedEncoder(
                parent,
                turningEncoder,
                mech);
        AngularPositionServo servo = new OutboardAngularPositionServo(
                parent,
                mech,
                combinedEncoder);

        servo.setProfile(profile);
        return servo;
    }

    private static AngularPositionServo getOnboard(
            SupplierLogger2 parent,
            SwerveKinodynamics kinodynamics,
            RotaryPositionSensor turningEncoder, Profile100 profile, RotaryMechanism mech) {
        PIDController turningPositionController = new PIDController(
                20, // kP
                0, // kI
                0, // kD
                dt);
        turningPositionController.enableContinuousInput(-Math.PI, Math.PI);
        turningPositionController.setTolerance(0.1, 0.1);
        AngularPositionServo servo = new OnboardAngularPositionServo(
                parent,
                mech,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                turningPositionController);
        servo.setProfile(profile);
        return servo;
    }

    private static RotaryPositionSensor turningEncoder(
            Class<? extends RotaryPositionSensor> encoderClass,
            SupplierLogger2 parent,
            int channel,
            double inputOffset,
            EncoderDrive drive) {
        if (encoderClass == AnalogTurningEncoder.class) {
            return new AnalogTurningEncoder(
                    parent,
                    channel,
                    inputOffset,
                    drive);
        }
        if (encoderClass == DutyCycleRotaryPositionSensor.class) {
            return new AS5048RotaryPositionSensor(
                    parent,
                    channel,
                    inputOffset,
                    drive);
        }
        throw new IllegalArgumentException("unknown encoder class: " + encoderClass.getName());

    }

    private WCPSwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        super(driveServo, turningServo);
        //
    }
}
