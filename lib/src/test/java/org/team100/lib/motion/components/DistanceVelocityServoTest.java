package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.MockMotor100;
import org.team100.lib.units.Distance100;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

class Distance100VelocityServoTest {
    @Test
    void testSimple() {

        String name = "test";
        MockMotor100<Distance100> driveMotor = new MockMotor100<>();
        MockEncoder100<Distance100> driveEncoder = new MockEncoder100<>();
        PIDController driveController = new PIDController(1, 0, 0);

        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 1, 1);

        SelectableVelocityServo<Distance100> servo = new SelectableVelocityServo<>(
                name,
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);

        Experiments.instance.testOverride(Experiment.UseClosedLoopVelocity, true);

        servo.setVelocity(0.5);
        assertEquals(0.5, driveMotor.velocity, 0.001);

        Experiments.instance.testOverride(Experiment.UseClosedLoopVelocity, false);

        servo.setVelocity(1.0);
        assertEquals(1, driveMotor.output, 0.001);
        servo.stop();
        assertEquals(0, driveMotor.output, 0.001);
    }
}
