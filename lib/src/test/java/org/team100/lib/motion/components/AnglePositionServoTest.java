package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
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
        OnboardPositionServo<Angle100> servo = new OnboardPositionServo<>(
                name,
                turningMotor,
                turningEncoder,
                1,
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
        MockEncoder100<Angle100> encoder = new MockEncoder100<>();

        OutboardPositionServo<Angle100> servo = new OutboardPositionServo<>(name, motor, encoder, Angle100.instance);
        servo.reset();
        servo.setPosition(1, 0);
        assertEquals(1, motor.position, kDelta);
    }
}
