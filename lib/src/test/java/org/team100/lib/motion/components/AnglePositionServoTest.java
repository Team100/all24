package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.motor.MockPositionMotor100;
import org.team100.lib.motor.MockVelocityMotor100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.units.Angle100;

import edu.wpi.first.math.controller.PIDController;

class AnglePositionServoTest {
    private static final double kDelta = 0.001;

    /** A minimal exercise. */
    @Test
    void testOnboard() {
        // long period to make the output bigger
        double period = 1;

        String name = "test";
        MockVelocityMotor100<Angle100> turningMotor = new MockVelocityMotor100<>();
        MockEncoder100<Angle100> turningEncoder = new MockEncoder100<>();

        PIDController turningController2 = new PIDController(1, 0, 0, period);

        Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        double maxVel = 1;
        OnboardPositionServo<Angle100> servo = new OnboardPositionServo<>(
                name,
                turningMotor,
                turningEncoder,
                maxVel,
                turningController2,
                profile,
                Angle100.instance);
        servo.reset();
        servo.setPosition(1, 0);
        assertEquals(0, turningMotor.output, 0.001);
        assertEquals(0.5, servo.getSetpoint().x(), kDelta);
        assertEquals(1.0, servo.getSetpoint().v(), kDelta);
        assertEquals(1, turningMotor.velocity, kDelta);
    }

    @Test
    void testOutboard() {
        String name = "test";
        MockPositionMotor100<Angle100> motor = new MockPositionMotor100<>();
        MockEncoder100<Angle100> externalEncoder = new MockEncoder100<>();
        MockEncoder100<Angle100> builtInEncoder = new MockEncoder100<>();
        CombinedEncoder<Angle100> combinedEncoder = new CombinedEncoder<>(
                externalEncoder, 1.0, builtInEncoder);
        Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);

        OutboardPositionServo<Angle100> servo = new OutboardPositionServo<>(
                name,
                motor,
                combinedEncoder,
                profile,
                Angle100.instance);
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
            System.out.println(motor.position);
        }
        assertEquals(1, motor.position, kDelta);
    }
}
