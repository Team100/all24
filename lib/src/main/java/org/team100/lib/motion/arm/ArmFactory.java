package org.team100.lib.motion.arm;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.arm.JointMotor;
import org.team100.lib.units.Angle;

public class ArmFactory {
    private static final String kArm = "/arm";
    private static final String kLower = "/arm/lower";
    private static final String kUpper = "/arm/upper";

    public static ArmSubsystem get() {
        Identity identity = Identity.get();
        switch (identity) {
            case TEST_BOARD_6B:
                // TODO: use the correct identity.
                return new ArmSubsystem(kArm,
                        new JointMotor(kLower, 4, 8),
                        new AnalogTurningEncoder(
                                kLower, 1, 0, 1,
                                AnalogTurningEncoder.Drive.DIRECT),
                        new JointMotor(kUpper, 30, 1),
                        new AnalogTurningEncoder(
                                kUpper, 0, 0, 1,
                                AnalogTurningEncoder.Drive.DIRECT));
            case BLANK:
                // for testing
                SimulatedMotor<Angle> lowerMotor = new SimulatedMotor<>(kLower);
                SimulatedMotor<Angle> upperMotor = new SimulatedMotor<>(kUpper);
                // note very high reduction ratio
                return new ArmSubsystem(kArm,
                        lowerMotor,
                        new SimulatedEncoder<>(new Angle() {
                        }, kLower, lowerMotor, 100),
                        upperMotor,
                        new SimulatedEncoder<>(new Angle() {
                        }, kUpper, upperMotor, 100));
            default:
                return new ArmSubsystem(kArm,
                        new JointMotor(kLower, 4, 8),
                        new AnalogTurningEncoder(
                                kLower, 1, 0, 1,
                                AnalogTurningEncoder.Drive.DIRECT),
                        new JointMotor(kUpper, 30, 1),
                        new AnalogTurningEncoder(
                                kUpper, 0, 0, 1,
                                AnalogTurningEncoder.Drive.DIRECT));
        }
    }

    private ArmFactory() {
        //
    }

}
