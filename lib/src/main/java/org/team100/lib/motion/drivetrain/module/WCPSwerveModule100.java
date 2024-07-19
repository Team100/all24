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
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.SimpleLinearMechanism;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motion.components.AngularPositionServo;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OnboardAngularPositionServo;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motion.components.OutboardAngularPositionServo;
import org.team100.lib.motion.components.SelectableAngularPositionServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.SupplierLogger;

import edu.wpi.first.math.controller.PIDController;

public class WCPSwerveModule100 extends SwerveModule100 {
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
            String name,
            SupplierLogger parent,
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
        SupplierLogger moduleLogger = parent.child(name);

        // TODO: revisit these constants
        PIDConstants drivePidConstants = new PIDConstants(.2); // .2
        PIDConstants turningPidConstants = new PIDConstants(.32); // 5
        Feedforward100 turningFF = Feedforward100.makeWCPSwerveTurningFalcon6();
        Feedforward100 driveFF = Feedforward100.makeWCPSwerveDriveFalcon6();

        LinearVelocityServo driveServo = driveServo(
                moduleLogger.child("Drive"),
                supplyLimitAmps,
                statorLimitAmps,
                driveMotorCanId,
                ratio,
                drivePidConstants,
                driveFF);

        AngularPositionServo turningServo = turningServo(
                moduleLogger.child("Turning"),
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

        return new WCPSwerveModule100(name, driveServo, turningServo);
    }

    private static LinearVelocityServo driveServo(
            SupplierLogger parent,
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
            SupplierLogger parent,
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
            SupplierLogger parent,
            SwerveKinodynamics kinodynamics,
            Falcon6Motor turningMotor,
            RotaryPositionSensor turningEncoder,
            double turningGearRatio,
            Profile100 profile) {

        Talon6Encoder builtInEncoder = new Talon6Encoder(
                parent,
                turningMotor);
                
        RotaryMechanism mech = new RotaryMechanism(
                turningMotor,
                builtInEncoder,
                turningGearRatio);

        AngularPositionServo outboard = getOutboard(
                parent,
                turningEncoder,
                profile,
                mech);

        OnboardAngularPositionServo onboard = getOnboard(
                parent,
                kinodynamics,
                turningEncoder,
                profile,
                mech);

        return new SelectableAngularPositionServo(
                outboard,
                onboard,
                () -> Experiments.instance.enabled(Experiment.OutboardSteering));
    }

    private static AngularPositionServo getOutboard(SupplierLogger parent, RotaryPositionSensor turningEncoder,
            Profile100 profile, RotaryMechanism mech) {
        // if we correct to exactly the primary reading, we effectively inject noise
        // into the secondary, so soften the response.
        final double primaryAuthority = 0.1;
        CombinedEncoder combinedEncoder = new CombinedEncoder(
                turningEncoder,
                primaryAuthority,
                mech);
        return new OutboardAngularPositionServo(
                parent,
                mech,
                combinedEncoder,
                profile);
    }

    private static OnboardAngularPositionServo getOnboard(
            SupplierLogger parent,
            SwerveKinodynamics kinodynamics,
            RotaryPositionSensor turningEncoder, Profile100 profile, RotaryMechanism mech) {
        PIDController turningPositionController = new PIDController(
                20, // kP
                0, // kI
                0, // kD
                dt);
        turningPositionController.enableContinuousInput(-Math.PI, Math.PI);
        turningPositionController.setTolerance(0.1, 0.1);
        return new OnboardAngularPositionServo(
                parent,
                mech,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                turningPositionController,
                profile);
    }

    private static RotaryPositionSensor turningEncoder(
            Class<? extends RotaryPositionSensor> encoderClass,
            SupplierLogger parent,
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
            String name,
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        super(name, driveServo, turningServo);
        //
    }
}
