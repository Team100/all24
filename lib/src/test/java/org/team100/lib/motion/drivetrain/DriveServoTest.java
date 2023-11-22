package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.drive.MockDriveEncoder;
import org.team100.lib.experiments.MockExperiments;
import org.team100.lib.motor.drive.MockDriveMotor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

class DriveServoTest {
    @Test
    void testSimple() {

        MockExperiments experiments = new MockExperiments();
        String name = "test";
        MockDriveMotor driveMotor = new MockDriveMotor();
        MockDriveEncoder driveEncoder = new MockDriveEncoder();
        PIDController driveController = new PIDController(1, 0, 0);

        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 1, 1);

        DriveServo servo = new DriveServo(
                experiments,
                name,
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);

        servo.setVelocity(0.5);
        assertEquals(0.5, driveMotor.velocity, 0.001);
        experiments.enablement = false;

        servo.setVelocity(1);
        assertEquals(1, driveMotor.output, 0.001);
        servo.set(0);
        assertEquals(0, driveMotor.output, 0.001);
    }
}
