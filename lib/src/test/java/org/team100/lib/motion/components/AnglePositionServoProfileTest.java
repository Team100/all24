package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.TestLogger;

import edu.wpi.first.math.controller.PIDController;

class AnglePositionServoProfileTest {
    private static final double kDelta = 0.001;
    private static final SupplierLogger2 logger = new TestLogger().getSupplierLogger();

    private final MockBareMotor motor;
    private final MockRotaryPositionSensor encoder;
    private final double period;
    private final PIDController controller2;
    private final AngularPositionServo servo;

    public AnglePositionServoProfileTest() {
        motor = new MockBareMotor();
        RotaryMechanism mech = new RotaryMechanism(
                logger,
                motor,
                new MockIncrementalBareEncoder(),
                1);
        encoder = new MockRotaryPositionSensor();
        period = 0.1;
        controller2 = new PIDController(1, 0, 0, period);
        controller2.enableContinuousInput(-Math.PI, Math.PI);

        Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        servo = new OnboardAngularPositionServo(
                logger,
                mech,
                encoder,
                1,
                controller2);
        servo.setProfile(profile);
        servo.reset();
    }

    /**
     * Profile invariant to support refactoring the servo. This is the WPILib
     * TrapezoidalProfile. This only works with
     * {@link org.team100.lib.experiments.Experiment.FilterFeedback} off.
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
        servo.setPosition(1, 0);
        assertEquals(motorVelocity, motor.velocity, kDelta);
        assertEquals(setpointPosition, servo.getSetpoint().x(), kDelta);
        assertEquals(setpointVelocity, servo.getSetpoint().v(), kDelta);
    }
}
