package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.experiments.MockExperiments;
import org.team100.lib.motor.MockMotor100;
import org.team100.lib.units.Angle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

class AngleVelocityServoTest {
    @Test
    void testSimple() {

        MockExperiments experiments = new MockExperiments();
        String name = "test";
        MockMotor100<Angle> motor = new MockMotor100<>();
        MockEncoder100<Angle> encoder = new MockEncoder100<>();
        PIDController controller = new PIDController(1, 0, 0);

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1, 1);

        VelocityServo<Angle> servo = new VelocityServo<>(
                experiments,
                name,
                motor,
                encoder,
                controller,
                feedforward);

        servo.setVelocity(0.5);
        assertEquals(0.5, motor.velocity, 0.001);
        experiments.enablement = false;

        servo.setVelocity(1.0);
        assertEquals(1, motor.output, 0.001);
        servo.stop();
        assertEquals(0, motor.output, 0.001);
    }
}
