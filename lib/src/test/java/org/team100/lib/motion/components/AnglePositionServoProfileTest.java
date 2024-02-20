package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.motor.MockMotor100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.units.Angle100;

import edu.wpi.first.math.controller.PIDController;

class AnglePositionServoProfileTest {
    private static final double kDelta = 0.001;

    private final String name;
    private final MockMotor100<Angle100> motor;
    private final MockEncoder100<Angle100> encoder;
    private final double period;
    private final PIDController controller2;
    private final PositionServo<Angle100> servo;

    public AnglePositionServoProfileTest() {
        name = "test";
        motor = new MockMotor100<>();
        encoder = new MockEncoder100<>();
        period = 0.1;
        controller2 = new PIDController(1, 0, 0, period);
        controller2.enableContinuousInput(-Math.PI, Math.PI);

        Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        servo = new PositionServo<>(
                name,
                motor,
                encoder,
                1,
                controller2,
                profile,
                Angle100.instance);
        servo.reset();
    }

    /**
     * Profile invariant to support refactoring the servo. This is the WPILib
     * TrapezoidalProfile.
     */
    @Test
    void testProfile() {

        verify(0.105, 0.005, 0.1);
        verify(0.209, 0.020, 0.2);
        verify(0.313, 0.045, 0.3);
        verify(0.417, 0.080, 0.4);
        verify(0.520, 0.125, 0.5);
        verify(0.623, 0.180, 0.6);
        verify(0.726, 0.245, 0.7);
        verify(0.828, 0.320, 0.8);
        verify(0.930, 0.405, 0.9);
        verify(1.000, 0.500, 1.0);
        verify(0.927, 0.595, 0.9);
        verify(0.819, 0.680, 0.8);
        verify(0.713, 0.755, 0.7);
        verify(0.606, 0.820, 0.6);
        verify(0.501, 0.875, 0.5);
        verify(0.395, 0.920, 0.4);
        verify(0.291, 0.955, 0.3);
        verify(0.187, 0.980, 0.2);
        verify(0.083, 0.995, 0.1);
        verify(-0.019, 1.000, 0.0);
        verify(-0.017, 1.000, 0.0);
    }

    private void verify(double motorVelocity, double setpointPosition, double setpointVelocity) {
        encoder.angle += motor.velocity * period;
        servo.setPosition(1);
        servo.periodic();
        assertEquals(motorVelocity, motor.velocity, kDelta);
        assertEquals(setpointPosition, servo.getSetpoint().x(), kDelta);
        assertEquals(setpointVelocity, servo.getSetpoint().v(), kDelta);
    }
}
