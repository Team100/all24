package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.drive.DriveEncoder;
import org.team100.lib.encoder.drive.MockDriveEncoder;
import org.team100.lib.experiments.MockExperiments;
import org.team100.lib.motor.drive.MockDriveMotor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

class ProfiledPIDPositionServoTest {
     @Test
    void testSimple() {
        
        MockExperiments experiments = new MockExperiments();
        String name = "test";
        MockDriveMotor turningMotor = new MockDriveMotor();
        DriveEncoder turningEncoder = new MockDriveEncoder();
        
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);
        // long period to make the output bigger
        ProfiledPIDController turningController = new ProfiledPIDController(1, 0, 0, constraints, 1);
        assertEquals(0.5, turningController.calculate(0, 1), 0.001);
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(1, 1, 1);

        ProfiledPIDPositionServo servo = new ProfiledPIDPositionServo(
                experiments,
                name,
                turningMotor,
                turningEncoder,
                turningController,
                turningFeedforward);

        servo.setPosition(1.0);
        assertEquals(0, turningMotor.output, 0.001);
        assertEquals(1, turningMotor.velocity, 0.001);

        experiments.enablement = false;
        servo.setPosition(1.0);
        assertEquals(1, turningMotor.output, 0.001);
        assertEquals(1, turningMotor.velocity, 0.001);

        servo.stop();
        assertEquals(0, turningMotor.output, 0.001);
        assertEquals(1, turningMotor.velocity, 0.001);

    }

    
}
