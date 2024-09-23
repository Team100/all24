package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.ProfileWPI;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;

class AngularPositionProfileTest implements Timeless {

    boolean dump = false;
    private static final double kDelta = 0.001;
    private static final SupplierLogger2 logger = new TestLogger().getSupplierLogger();

    private final MockBareMotor motor;
    private final RotaryMechanism mech;
    private final MockRotaryPositionSensor encoder;
    private final double period;
    private final PIDController controller2;

    private AngularPositionServo servo;

    public AngularPositionProfileTest() {
        motor = new MockBareMotor();
        mech = new RotaryMechanism(
                logger,
                motor,
                new MockIncrementalBareEncoder(),
                1);
        encoder = new MockRotaryPositionSensor();
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
        servo = new OnboardAngularPositionServo(
                logger,
                mech,
                encoder,
                1,
                controller2);
        servo.setProfile(profile);
        servo.reset();

        verifyTrapezoid();
    }

    @Test
    void testProfile() {
        Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        servo = new OnboardAngularPositionServo(
                logger,
                mech,
                encoder,
                1,
                controller2);
        servo.setProfile(profile);
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
