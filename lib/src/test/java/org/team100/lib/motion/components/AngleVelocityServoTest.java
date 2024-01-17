package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.MockMotor100;
import org.team100.lib.units.Angle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

class AngleVelocityServoTest {
    @Test
    void testSimple() {

        String name = "test";
        MockMotor100<Angle> motor = new MockMotor100<>();
        MockEncoder100<Angle> encoder = new MockEncoder100<>();
        PIDController controller = new PIDController(1, 0, 0);

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1, 1);

        SelectableVelocityServo<Angle> servo = new SelectableVelocityServo<>(
                name,
                motor,
                encoder,
                controller,
                feedforward);

        Experiments.instance.testOverride(Experiment.UseClosedLoopVelocity, true);

        servo.setVelocity(0.5);
        assertEquals(0.5, motor.velocity, 0.001);
        
        Experiments.instance.testOverride(Experiment.UseClosedLoopVelocity, false);

        servo.setVelocity(1.0);
        assertEquals(1, motor.output, 0.001);
        servo.stop();
        assertEquals(0, motor.output, 0.001);
    }
}
