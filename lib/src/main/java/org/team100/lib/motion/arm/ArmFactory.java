package org.team100.lib.motion.arm;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.encoder.turning.EncoderDrive;
import org.team100.lib.motor.DutyCycleMotor100;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.arm.JointMotor;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.units.Angle100;

/**
 * Produces real or simulated arm subsystems depending on identity.
 */
public class ArmFactory {
    private static final String kArm = "arm";
    private static final String kLower = "arm/lower";
    private static final String kUpper = "arm/upper";

    public static ArmSubsystem get(Logger parent) {
        switch (Identity.instance) {
            case TEST_BOARD_6B:
                return real(parent);
            case BLANK:
                // for testing
                return simulated(parent);
            default:
                return simulated(parent);
        }
    }

    private static ArmSubsystem real(Logger parent) {
        final double kLowerEncoderOffset = 0.861614;
        final double kUpperEncoderOffset = 0.266396;

        DutyCycleMotor100 lowerMotor = new JointMotor(parent.child(kLower), 4, 8);
        // NOTE: the encoder inversion used to be in the subsystem,
        // but now it is here.
        RotaryPositionSensor lowerEncoder = new AnalogTurningEncoder(
                parent.child(kLower),
                1, // analog input 1
                kLowerEncoderOffset,
                EncoderDrive.INVERSE);

        DutyCycleMotor100 upperMotor = new JointMotor(parent.child(kUpper), 30, 1);
        RotaryPositionSensor upperEncoder = new AnalogTurningEncoder(
                parent.child(kUpper),
                0, // analog input 0
                kUpperEncoderOffset,
                EncoderDrive.DIRECT);

        return new ArmSubsystem(
                parent.child(kArm),
                lowerMotor,
                lowerEncoder,
                upperMotor,
                upperEncoder);
    }

    private static ArmSubsystem simulated(Logger parent) {
        // for testing
        // note very high reduction ratio
        // motor speed is rad/s

        SimulatedMotor<Angle100> lowerMotor = new SimulatedMotor<>(parent.child(kLower), 600);
        // limits used to be -1, 1, when we used winding encoders.
        // i don't think we ever actually *use* the limits for anything, though.
        SimulatedRotaryPositionSensor lowerEncoder = new SimulatedRotaryPositionSensor(
                parent.child(kLower), lowerMotor, 200);

        SimulatedMotor<Angle100> upperMotor = new SimulatedMotor<>(parent.child(kUpper), 600);
        // limits used to be 0.1, 2.5, when we used winding encoders.
        // i don't think we ever actually *use* the limits for anything, though.
        SimulatedRotaryPositionSensor upperEncoder = new SimulatedRotaryPositionSensor(
                parent.child(kUpper), upperMotor, 200);
        return new ArmSubsystem(
                parent.child(kArm),
                lowerMotor,
                lowerEncoder,
                upperMotor,
                upperEncoder);
    }

    private ArmFactory() {
        //
    }

}
