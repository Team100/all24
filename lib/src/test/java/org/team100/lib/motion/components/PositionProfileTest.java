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
        controller2 = new PIDController(5, 0, 0, period);
        feedforward = new SimpleMotorFeedforward(1, 1, 1);
    }

    /**
     * Profile invariant to support refactoring the servo. This is the WPILib
     * TrapezoidalProfile.
     */
    @Test
    void testTrapezoid() {
        // TODO: tune this
        PIDController vController = new PIDController(1, 0, 0, period);
        VelocityServo<Distance> vServo = new VelocityServo<>(
                experiments,
                name,
                motor,
                encoder,
                vController,
                feedforward);
        ChoosableProfile profile = new ChoosableProfile(1, 1, ChoosableProfile.Mode.TRAPEZOID);
        servo = new PositionServo<>(
                name,
                vServo,
                encoder,
                1,
                controller2,
                profile,
                Distance.instance);
        servo.reset();

        verifyTrapezoid();
    }

    @Test
    void testProfile() {
        // TODO: tune this
        PIDController vController = new PIDController(1, 0, 0, period);
        VelocityServo<Distance> vServo = new VelocityServo<>(
                experiments,
                name,
                motor,
                encoder,
                vController,
                feedforward);
        ChoosableProfile profile = new ChoosableProfile(1, 1, ChoosableProfile.Mode.TRAPEZOID);
        servo = new PositionServo<>(
                name,
                vServo,
                encoder,
                1,
                controller2,
                profile,
                Distance.instance);
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

    @Test
    void testExponential() {
        // TODO: tune this
        PIDController vController = new PIDController(5, 0, 0, period);
        VelocityServo<Distance> vServo = new VelocityServo<>(
                experiments,
                name,
                motor,
                encoder,
                vController,
                feedforward);
        ChoosableProfile profile = new ChoosableProfile(1, 1, ChoosableProfile.Mode.EXPONENTIAL);
        servo = new PositionServo<>(
                name,
                vServo,
                encoder,
                1,
                controller2,
                profile,
                Distance.instance);
        servo.reset();
        verifyExp();
    }

    private void verifyExp() {
        verify(0.228, 0.009, 0.181);
        verify(0.391, 0.035, 0.330);
        verify(0.513, 0.074, 0.451);
        verify(0.608, 0.125, 0.551);
        verify(0.682, 0.184, 0.632);
        verify(0.741, 0.251, 0.699);
        verify(0.788, 0.323, 0.753);
        verify(0.827, 0.401, 0.798);
        verify(0.859, 0.483, 0.835);
        verify(0.884, 0.568, 0.865);
        verify(0.905, 0.655, 0.889);
        verify(0.923, 0.745, 0.909);
        verify(0.937, 0.837, 0.926);
        verify(0.634, 0.921, 0.673);
        verify(0.272, 0.972, 0.370);
        verify(0.009, 0.997, 0.122);
        verify(-0.100, 1.000, 0.000);
        verify(-0.050, 1.000, 0.000);
        verify(-0.025, 1.000, 0.000);
        verify(-0.013, 1.000, 0.000);
        verify(-0.006, 1.000, 0.000);
    }

    private void verify(double motorVelocity, double setpointPosition, double setpointVelocity) {
        encoder.angle += motor.velocity * period;
        servo.setPosition(1);
        // useful to fix up the examples above
        // System.out.printf("verify(%5.3f, %5.3f, %5.3f);\n", motor.velocity,
        // servo.getSetpoint().position, servo.getSetpoint().velocity);
        // assertEquals(motorVelocity, motor.velocity, kDelta);
        assertEquals(setpointPosition, servo.getSetpoint().position, kDelta);
        assertEquals(setpointVelocity, servo.getSetpoint().velocity, kDelta);
    }
}
