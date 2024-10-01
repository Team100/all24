package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.TestSupplierLogger;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.math.controller.PIDController;

class AnglePositionServoProfileTest {
    private static final double kDelta = 0.001;
    private static final SupplierLogger2 logger = new TestSupplierLogger(new TestPrimitiveLogger());

    private final MockBareMotor motor;
    private final MockRotaryPositionSensor encoder;
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
        controller2 = new PIDController(1, 0, 0, TimedRobot100.LOOP_PERIOD_S);
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
     * Note: i modified the expectation a bit when i imposed the 0.02s clock
     */
    @Test
    void testProfile() {

        verify(0.101, 0.005, 0.1);
        verify(0.202, 0.020, 0.2);
        verify(0.303, 0.045, 0.3);
        verify(0.403, 0.080, 0.4);
        verify(0.504, 0.125, 0.5);
        verify(0.605, 0.180, 0.6);
        verify(0.705, 0.245, 0.7);
        verify(0.805, 0.320, 0.8);
        verify(0.905, 0.405, 0.9);
        verify(1.000, 0.500, 1.0);
        verify(0.905, 0.595, 0.9);
        verify(0.803, 0.680, 0.8);
        verify(0.702, 0.755, 0.7);
        verify(0.601, 0.820, 0.6);
        verify(0.499, 0.875, 0.5);
        verify(0.399, 0.920, 0.4);
        verify(0.298, 0.955, 0.3);
        verify(0.197, 0.980, 0.2);
        verify(0.097, 0.995, 0.1);
        verify(-0.003, 1.000, 0.0);
        verify(-0.003, 1.000, 0.0);
    }

    private void verify(double motorVelocity, double setpointPosition, double setpointVelocity) {
        // spin for 100ms
        for (int i = 0; i < 5; ++i) {
            encoder.angle += motor.velocity * TimedRobot100.LOOP_PERIOD_S;
            servo.setPosition(1, 0);
        }
        assertEquals(motorVelocity, motor.velocity, kDelta);
        assertEquals(setpointPosition, servo.getSetpoint().x(), kDelta);
        assertEquals(setpointVelocity, servo.getSetpoint().v(), kDelta);
    }
}
