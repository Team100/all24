package org.team100.lib.motion.arm;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.arm.JointMotor;
import org.team100.lib.units.Angle;

/**
 * Produces real or simulated arm subsystems depending on identity.
 */
public class ArmFactory {
    private static final String kArm = "arm";
    private static final String kLower = "arm/lower";
    private static final String kUpper = "arm/upper";

    public static ArmSubsystem get() {
        switch (Identity.instance) {
            case TEST_BOARD_6B:
                return real();
            case BLANK:
                // for testing
                return simulated();
            default:
                return real();
        }
    }

    private static ArmSubsystem real() {
        Motor100<Angle> lowerMotor = new JointMotor(kLower, 4, 8);
        // NOTE: the encoder inversion used to be in the subsystem,
        // but now it is here.
        Encoder100<Angle> lowerEncoder = new AnalogTurningEncoder(
                kLower, 1, 0, 1, AnalogTurningEncoder.Drive.INVERSE);
        Motor100<Angle> upperMotor = new JointMotor(kUpper, 30, 1);
        Encoder100<Angle> upperEncoder = new AnalogTurningEncoder(
                kUpper, 0, 0, 1, AnalogTurningEncoder.Drive.DIRECT);
        double kLowerEncoderZero = 0.861614;
        double kUpperEncoderZero = 0.266396;
        return new ArmSubsystem(
                kArm,
                lowerMotor,
                lowerEncoder,
                upperMotor,
                upperEncoder,
                kLowerEncoderZero,
                kUpperEncoderZero);
    }

    private static ArmSubsystem simulated() {
        // for testing
        // note very high reduction ratio
        SimulatedMotor<Angle> lowerMotor = new SimulatedMotor<>(kLower);
        SimulatedEncoder<Angle> lowerEncoder = new SimulatedEncoder<>(
                Angle.instance, kLower, lowerMotor, 25);
        SimulatedMotor<Angle> upperMotor = new SimulatedMotor<>(kUpper);
        SimulatedEncoder<Angle> upperEncoder = new SimulatedEncoder<>(
                Angle.instance, kUpper, upperMotor, 25);
        // simulated encoders are correctly aligned
        double kLowerEncoderZero = 0;
        double kUpperEncoderZero = 0;
        return new ArmSubsystem(
                kArm,
                lowerMotor,
                lowerEncoder,
                upperMotor,
                upperEncoder,
                kLowerEncoderZero,
                kUpperEncoderZero);
    }

    private ArmFactory() {
        //
    }

}
