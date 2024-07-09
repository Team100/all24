package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;

class AnglePositionServoTest {
    private static final double kDelta = 0.001;
    private static final Logger logger = new TestLogger();
    private static final boolean kActuallyPrint = false;

    /** A minimal exercise. */
    @Test
    void testOnboard() {
        // long period to make the output bigger
        double period = 1;

        MockBareMotor turningMotor = new MockBareMotor();
        RotaryMechanism mech = new RotaryMechanism(
                turningMotor,
                new MockIncrementalBareEncoder(),
                1);
        MockRotaryPositionSensor turningEncoder = new MockRotaryPositionSensor();

        PIDController turningController2 = new PIDController(1, 0, 0, period);

        Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        double maxVel = 1;
        OnboardAngularPositionServo servo = new OnboardAngularPositionServo(
                logger,
                mech,
                turningEncoder,
                maxVel,
                turningController2,
                profile);
        servo.reset();
        servo.setPosition(1, 0);
        assertEquals(0, turningMotor.output, 0.001);
        assertEquals(0.5, servo.getSetpoint().x(), kDelta);
        assertEquals(1.0, servo.getSetpoint().v(), kDelta);
        assertEquals(1, turningMotor.velocity, kDelta);
    }

    @Test
    void testOutboard() {
        MockBareMotor motor = new MockBareMotor();
        RotaryMechanism mech = new RotaryMechanism(
                motor,
                new MockIncrementalBareEncoder(),
                1);
        MockRotaryPositionSensor externalEncoder = new MockRotaryPositionSensor();
        CombinedEncoder combinedEncoder = new CombinedEncoder(
                externalEncoder, 1.0, mech);
        Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);

        OutboardPositionServo servo = new OutboardPositionServo(
                logger,
                mech,
                combinedEncoder,
                profile);
        servo.reset();
        // it moves slowly
        servo.setPosition(1, 0);
        assertEquals(2e-4, motor.position, 1e-4);
        servo.setPosition(1, 0);
        assertEquals(8e-4, motor.position, 1e-4);
        servo.setPosition(1, 0);
        assertEquals(0.002, motor.position, kDelta);
        for (int i = 0; i < 100; ++i) {
            // run it for awhile
            servo.setPosition(1, 0);
            if (kActuallyPrint)
                Util.printf("%5.3f\n", motor.position);
        }
        assertEquals(1, motor.position, kDelta);
    }
}
