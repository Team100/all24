package org.team100.lib.motion.arm;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.encoder.turning.EncoderDrive;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.arm.JointMotor;
import org.team100.lib.units.Angle100;

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
                return simulated();
        }
    }

    private static ArmSubsystem real() {
        final double kLowerEncoderOffset = 0.861614;
        final double kUpperEncoderOffset = 0.266396;

        Motor100<Angle100> lowerMotor = new JointMotor(kLower, 4, 8);
        // NOTE: the encoder inversion used to be in the subsystem,
        // but now it is here.
        Encoder100<Angle100> lowerEncoder = new AnalogTurningEncoder(
                kLower,
                1, // analog input 1
                kLowerEncoderOffset,
                1, // encoder is 1:1 with the arm joint
                EncoderDrive.INVERSE);

        Motor100<Angle100> upperMotor = new JointMotor(kUpper, 30, 1);
        Encoder100<Angle100> upperEncoder = new AnalogTurningEncoder(
                kUpper,
                0, // analog input 0
                kUpperEncoderOffset,
                1, // encoder is 1:1 with the arm joint
                EncoderDrive.DIRECT);

        return new ArmSubsystem(
                kArm,
                lowerMotor,
                lowerEncoder,
                upperMotor,
                upperEncoder);
    }

    private static ArmSubsystem simulated() {
        // for testing
        // note very high reduction ratio
        // motor speed is rad/s

        SimulatedMotor<Angle100> lowerMotor = new SimulatedMotor<>(kLower, 600);
        SimulatedEncoder<Angle100> lowerEncoder = new SimulatedEncoder<>(
                kLower, lowerMotor, 200, -1, 1);

        SimulatedMotor<Angle100> upperMotor = new SimulatedMotor<>(kUpper, 600);
        SimulatedEncoder<Angle100> upperEncoder = new SimulatedEncoder<>(
                kUpper, upperMotor, 200, 0.1, 2.5);
        return new ArmSubsystem(
                kArm,
                lowerMotor,
                lowerEncoder,
                upperMotor,
                upperEncoder);
    }

    private ArmFactory() {
        //
    }

}
