package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockTurningEncoder;
import org.team100.lib.encoder.turning.TurningEncoder;
import org.team100.lib.experiments.MockExperiments;
import org.team100.lib.motor.turning.MockTurningMotor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** A minimal exercise. */
class TurningServoTest {

    @Test
    void testSimple() {
        
        MockExperiments experiments = new MockExperiments();
        String name = "test";
        MockTurningMotor turningMotor = new MockTurningMotor();
        TurningEncoder turningEncoder = new MockTurningEncoder();
        
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);
        // long period to make the output bigger
        ProfiledPIDController turningController = new ProfiledPIDController(1, 0, 0, constraints, 1);
        assertEquals(0.5, turningController.calculate(0, 1), 0.001);
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(1, 1, 1);

        TurningServo servo = new TurningServo(
                experiments,
                name,
                turningMotor,
                turningEncoder,
                turningController,
                turningFeedforward);

        servo.setAngle(new Rotation2d(1));
        assertEquals(0, turningMotor.turningOutput, 0.001);
        assertEquals(1, turningMotor.turningVelocity, 0.001);

        experiments.enablement = false;
        servo.setAngle(new Rotation2d(1));
        assertEquals(1, turningMotor.turningOutput, 0.001);
        assertEquals(1, turningMotor.turningVelocity, 0.001);

        servo.set(0);
        assertEquals(0, turningMotor.turningOutput, 0.001);
        assertEquals(1, turningMotor.turningVelocity, 0.001);

    }

}
