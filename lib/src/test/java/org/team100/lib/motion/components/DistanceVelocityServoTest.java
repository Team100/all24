package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.experiments.MockExperiments;
import org.team100.lib.motor.MockMotor100;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

class DistanceVelocityServoTest {
    @Test
    void testSimple() {

        MockExperiments experiments = new MockExperiments();
        String name = "test";
        MockMotor100<Distance> driveMotor = new MockMotor100<>();
        MockEncoder100<Distance> driveEncoder = new MockEncoder100<>();
        PIDController driveController = new PIDController(1, 0, 0);

        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 1, 1);

        VelocityServo<Distance> servo = new VelocityServo<>(
                experiments,
                name,
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);

        servo.setVelocity(0.5);
        assertEquals(0.5, driveMotor.velocity, 0.001);
        experiments.enablement = false;

        servo.setVelocity(1.0);
        assertEquals(1, driveMotor.output, 0.001);
        servo.stop();
        assertEquals(0, driveMotor.output, 0.001);
    }
}
