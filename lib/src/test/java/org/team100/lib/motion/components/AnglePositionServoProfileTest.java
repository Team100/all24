package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockTurningEncoder;
import org.team100.lib.experiments.MockExperiments;
import org.team100.lib.motor.MockMotor100;
import org.team100.lib.units.Angle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

class AnglePositionServoProfileTest {
    private static final double kDelta = 0.001;

    private final MockExperiments experiments;
    private final String name;
    private final MockMotor100<Angle> motor;
    private final MockTurningEncoder encoder;
    private final TrapezoidProfile.Constraints constraints;
    private final double period;
    private final PIDController controller2;
    private final SimpleMotorFeedforward feedforward;
    private final AnglePositionServo servo;

    public AnglePositionServoProfileTest() {
        experiments = new MockExperiments();
        name = "test";
        motor = new MockMotor100<>();
        encoder = new MockTurningEncoder();
        constraints = new TrapezoidProfile.Constraints(1, 1);
        period = 0.1;
        controller2 = new PIDController(1, 0, 0, period);
        controller2.enableContinuousInput(0, 2 * Math.PI);
        feedforward = new SimpleMotorFeedforward(1, 1, 1);

        // TODO: tune this
        PIDController angleVelocityController = new PIDController(1, 0, 0, period);
        VelocityServo<Angle> turningVelocityServo = new VelocityServo<>(
                experiments,
                name,
                motor,
                encoder,
                angleVelocityController,
                feedforward);

        servo = new AnglePositionServo(
                name,
                turningVelocityServo,
                encoder,
                constraints,
                controller2);
    }

    /**
     * Profile invariant to support refactoring the servo. This is the WPILib
     * TrapezoidalProfile.
     */
    @Test
    void testProfile() {
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

    private void verify(double motorVelocity, double setpointPosition, double setpointVelocity) {
        encoder.angle += motor.velocity * period;
        servo.setPosition(new Rotation2d(1));
        assertEquals(motorVelocity, motor.velocity, kDelta);
        assertEquals(setpointPosition, servo.getSetpoint().position, kDelta);
        assertEquals(setpointVelocity, servo.getSetpoint().velocity, kDelta);
    }
}
