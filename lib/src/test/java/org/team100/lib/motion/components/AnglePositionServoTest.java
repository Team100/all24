package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.MockMotor100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.units.Angle100;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

class Angle100PositionServoTest {
    private static final double kDelta = 0.001;

    /** A minimal exercise. */
    @Test
    void testSimple() {
        // long period to make the output bigger
        double period = 1;

        String name = "test";
        MockMotor100<Angle100> turningMotor = new MockMotor100<>();
        MockEncoder100<Angle100> turningEncoder = new MockEncoder100<>();


        PIDController turningController2 = new PIDController(1, 0, 0, period);
        // turningController2.enableContinuousInput(0, 2 * Math.PI);

        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(1, 1, 1);

        PIDController angle100Angle100VelocityController = new PIDController(1, 0, 0, period);
        SelectableVelocityServo<Angle100> turningVelocityServo = new SelectableVelocityServo<>(
                name,
                turningMotor,
                turningEncoder,
                angle100Angle100VelocityController,
                turningFeedforward);

        Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        PositionServo<Angle100> servo = new PositionServo<>(
                name,
                turningVelocityServo,
                turningEncoder,
                1,
                turningController2,
                profile,
                Angle100.instance);
        servo.reset();
        servo.setPosition(1);
        servo.periodic();
        assertEquals(0, turningMotor.output, 0.001);
        assertEquals(0.5, servo.getSetpoint().x(), kDelta);
        assertEquals(1.0, servo.getSetpoint().v(), kDelta);
        assertEquals(1, turningMotor.velocity, kDelta);

        Experiments.instance.testOverride(Experiment.UseClosedLoopVelocity, false);

        servo.setPosition(1);
        servo.periodic();
        assertEquals(1.0, turningMotor.output, kDelta);
        assertEquals(1.0, turningMotor.velocity, kDelta);

        servo.stop();
        assertEquals(0, turningMotor.output, kDelta);
        assertEquals(1.0, turningMotor.velocity, kDelta);
    }
}
