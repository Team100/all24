package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.drive.MockDriveEncoder;
import org.team100.lib.experiments.MockExperiments;
import org.team100.lib.motor.MockMotor100;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

class ProfiledPIDPositionServoTest {
     @Test
    void testSimple() {
        
        MockExperiments experiments = new MockExperiments();
        String name = "test";
        MockMotor100<Distance> motor = new MockMotor100<>();
        MockDriveEncoder turningEncoder = new MockDriveEncoder();
        
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);
        // long period to make the output bigger
        ProfiledPIDController turningController = new ProfiledPIDController(1, 0, 0, constraints, 1);
        assertEquals(0.5, turningController.calculate(0, 1), 0.001);
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(1, 1, 1);

        ProfiledPIDPositionServo servo = new ProfiledPIDPositionServo(
                experiments,
                name,
                motor,
                turningEncoder,
                turningController,
                turningFeedforward);

        servo.setPosition(1.0);
        assertEquals(0, motor.output, 0.001);
        assertEquals(1, motor.velocity, 0.001);

        experiments.enablement = false;
        servo.setPosition(1.0);
        assertEquals(1, motor.output, 0.001);
        assertEquals(1, motor.velocity, 0.001);

        servo.stop();
        assertEquals(0, motor.output, 0.001);
        assertEquals(1, motor.velocity, 0.001);

    }

    
}
