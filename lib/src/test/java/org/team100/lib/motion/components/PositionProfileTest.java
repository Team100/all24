package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.MockEncoder100;
import org.team100.lib.motor.MockVelocityMotor100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.ProfileWPI;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.testing.Timeless;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;

class PositionProfileTest implements Timeless {
    boolean dump = false;
    private static final double kDelta = 0.001;
    private static final Logger logger = new TestLogger();

    private final MockVelocityMotor100<Distance100> motor;
    private final MockEncoder100<Distance100> encoder;
    private final double period;
    private final PIDController controller2;
    private OnboardPositionServo<Distance100> servo;

    public PositionProfileTest() {
        motor = new MockVelocityMotor100<>();
        encoder = new MockEncoder100<>();
        period = 0.1;
        controller2 = new PIDController(5, 0, 0, period);
    }

    /**
     * Profile invariant to support refactoring the servo. This is the WPILib
     * TrapezoidalProfile.
     */
    @Test
    void testTrapezoid() {
        Profile100 profile = new ProfileWPI(1, 1);
        servo = new OnboardPositionServo<>(
                logger,
                motor,
                encoder,
                1,
                controller2,
                profile,
                Distance100.instance);
        servo.reset();

        verifyTrapezoid();
    }

    @Test
    void testProfile() {
        Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        servo = new OnboardPositionServo<>(
                logger,
                motor,
                encoder,
                1,
                controller2,
                profile,
                Distance100.instance);
        servo.reset();
        verifyTrapezoid();
    }

    private void verifyTrapezoid() {
        verify(0.125, 0.005, 0.100);
        verify(0.238, 0.020, 0.200);
        verify(0.344, 0.045, 0.300);
        verify(0.447, 0.080, 0.400);
        verify(0.548, 0.125, 0.500);
        verify(0.649, 0.180, 0.600);
        verify(0.750, 0.245, 0.700);
        verify(0.850, 0.320, 0.800);
        verify(0.950, 0.405, 0.900);
        verify(1.000, 0.500, 1.000);
        verify(0.925, 0.595, 0.900);
        verify(0.787, 0.680, 0.800);
        verify(0.669, 0.755, 0.700);
        verify(0.559, 0.820, 0.600);
        verify(0.455, 0.875, 0.500);
        verify(0.352, 0.920, 0.400);
        verify(0.251, 0.955, 0.300);
        verify(0.151, 0.980, 0.200);
        verify(0.050, 0.995, 0.100);
        verify(-0.050, 1.000, 0.000);
        verify(-0.025, 1.000, 0.000);
    }

    private void verify(double motorVelocity, double setpointPosition, double setpointVelocity) {
        encoder.angle += motor.velocity * period;
        servo.setPosition(1, 0);
        stepTime(0.02);
        // useful to fix up the examples above
        if (dump)
            Util.printf("verify(%5.3f, %5.3f, %5.3f);\n", motor.velocity,
                    servo.getSetpoint().x(), servo.getSetpoint().v());
        assertEquals(setpointPosition, servo.getSetpoint().x(), kDelta);
        assertEquals(setpointVelocity, servo.getSetpoint().v(), kDelta);
    }
}
