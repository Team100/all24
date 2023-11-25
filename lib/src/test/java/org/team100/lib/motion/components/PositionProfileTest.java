package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.experiments.MockExperiments;
import org.team100.lib.motor.MockMotor100;
import org.team100.lib.profile.ChoosableProfile;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

class PositionProfileTest {
    private static final double kDelta = 0.001;

    private final MockExperiments experiments;
    private final String name;
    private final MockMotor100<Distance> motor;
    private final MockEncoder100<Distance> encoder;
    private final double period;
    private final PIDController controller2;
    private final SimpleMotorFeedforward feedforward;
    private PositionServo<Distance> servo;

    public PositionProfileTest() {
        experiments = new MockExperiments();
        name = "test";
        motor = new MockMotor100<>();
        encoder = new MockEncoder100<>();
        period = 0.1;
        controller2 = new PIDController(1, 0, 0, period);
        feedforward = new SimpleMotorFeedforward(1, 1, 1);
    }

    /**
     * Profile invariant to support refactoring the servo. This is the WPILib
     * TrapezoidalProfile.
     */
    @Test
    void testTrapezoid() {
        // TODO: tune this
        PIDController angleVelocityController = new PIDController(1, 0, 0, period);
        VelocityServo<Distance> turningVelocityServo = new VelocityServo<>(
                experiments,
                name,
                motor,
                encoder,
                angleVelocityController,
                feedforward);
        ChoosableProfile profile = new ChoosableProfile(1, 1, 0, ChoosableProfile.Mode.TRAPEZOID);
        servo = new PositionServo<>(
                name,
                turningVelocityServo,
                encoder,
                1,
                controller2,
                profile,
                x -> x);

        verifyTrapezoid();
    }

    /**
     * Verifies that the MotionProfile in infinite-jerk mode is the same
     * as the WPILib trapezoid.
     */
    @Test
    void testProfile() {
        // TODO: tune this
        PIDController angleVelocityController = new PIDController(1, 0, 0, period);
        VelocityServo<Distance> turningVelocityServo = new VelocityServo<>(
                experiments,
                name,
                motor,
                encoder,
                angleVelocityController,
                feedforward);
        ChoosableProfile profile = new ChoosableProfile(1, 1, 0, ChoosableProfile.Mode.MOTION_PROFILE);
        servo = new PositionServo<>(
                name,
                turningVelocityServo,
                encoder,
                1,
                controller2,
                profile,
                x -> x);
        verifyTrapezoid();
    }

    private void verifyTrapezoid() {
        verify(0.077, 0.005, 0.1);
        verify(0.187, 0.020, 0.2);
        verify(0.297, 0.045, 0.3);
        verify(0.406, 0.080, 0.4);
        verify(0.513, 0.125, 0.5);
        verify(0.620, 0.180, 0.6);
        verify(0.726, 0.245, 0.7);
        verify(0.832, 0.320, 0.8);
        verify(0.937, 0.405, 0.9);
        verify(1.000, 0.500, 1.0);
        verify(0.933, 0.595, 0.9);
        verify(0.821, 0.680, 0.8);
        verify(0.711, 0.755, 0.7);
        verify(0.601, 0.820, 0.6);
        verify(0.493, 0.875, 0.5);
        verify(0.385, 0.920, 0.4);
        verify(0.279, 0.955, 0.3);
        verify(0.173, 0.980, 0.2);
        verify(0.067, 0.995, 0.1);
        verify(0.000, 1.000, 0.0);
        verify(0.000, 1.000, 0.0);
    }

    /**
     * With finite jerk, it's different.
     */
    @Test
    void testSProfile() {
        // TODO: tune this
        PIDController angleVelocityController = new PIDController(1, 0, 0, period);
        VelocityServo<Distance> turningVelocityServo = new VelocityServo<>(
                experiments,
                name,
                motor,
                encoder,
                angleVelocityController,
                feedforward);
        ChoosableProfile profile = new ChoosableProfile(1, 1, 100, ChoosableProfile.Mode.MOTION_PROFILE);
        servo = new PositionServo<>(
                name,
                turningVelocityServo,
                encoder,
                1,
                controller2,
                profile,
                x -> x);
        verifySProfile();
    }

    private void verifySProfile() {
        verify(0.072, 0.004, 0.095);
        verify(0.176, 0.018, 0.190);
        verify(0.280, 0.042, 0.285);
        verify(0.384, 0.075, 0.380);
        verify(0.486, 0.117, 0.475);
        verify(0.587, 0.170, 0.570);
        verify(0.688, 0.231, 0.665);
        verify(0.788, 0.302, 0.760);
        verify(0.888, 0.383, 0.855);
        verify(0.987, 0.473, 0.950);
        verify(0.962, 0.568, 0.928);
        verify(0.851, 0.656, 0.828);
        verify(0.740, 0.734, 0.728);
        verify(0.631, 0.802, 0.628);
        verify(0.522, 0.860, 0.528);
        verify(0.414, 0.908, 0.428);
        verify(0.308, 0.946, 0.328);
        verify(0.202, 0.973, 0.228);
        verify(0.096, 0.992, 0.128);
        verify(0.000, 0.999, 0.028);
        verify(0.000, 1.000, 0.000);
    }

    // TODO: make this pass
    // @Test
    void testExponential() {
        // TODO: tune this
        PIDController angleVelocityController = new PIDController(1, 0, 0, period);
        VelocityServo<Distance> turningVelocityServo = new VelocityServo<>(
                experiments,
                name,
                motor,
                encoder,
                angleVelocityController,
                feedforward);
        ChoosableProfile profile = new ChoosableProfile(1, 1, 100, ChoosableProfile.Mode.EXPONENTIAL);
        servo = new PositionServo<>(
                name,
                turningVelocityServo,
                encoder,
                1,
                controller2,
                profile,
                x -> x);
        verifySProfile();
    }

    private void verify(double motorVelocity, double setpointPosition, double setpointVelocity) {
        encoder.angle += motor.velocity * period;
        servo.setPosition(1);
        assertEquals(motorVelocity, motor.velocity, kDelta);
        assertEquals(setpointPosition, servo.getSetpoint().position, kDelta);
        assertEquals(setpointVelocity, servo.getSetpoint().velocity, kDelta);
    }
}
