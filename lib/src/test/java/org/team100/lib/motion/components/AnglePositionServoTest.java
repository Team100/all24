package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.MockMotor100;
import org.team100.lib.profile.ChoosableProfile;
import org.team100.lib.units.Angle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

class AnglePositionServoTest {
    private static final double kDelta = 0.001;

    /** A minimal exercise. */
    @Test
    void testSimple() {
        // long period to make the output bigger
        double period = 1;

        String name = "test";
        MockMotor100<Angle> turningMotor = new MockMotor100<>();
        MockEncoder100<Angle> turningEncoder = new MockEncoder100<>();


        PIDController turningController2 = new PIDController(1, 0, 0, period);
        // turningController2.enableContinuousInput(0, 2 * Math.PI);

        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(1, 1, 1);

        PIDController angleVelocityController = new PIDController(1, 0, 0, period);
        VelocityServo<Angle> turningVelocityServo = new VelocityServo<>(
                name,
                turningMotor,
                turningEncoder,
                angleVelocityController,
                turningFeedforward);

        ChoosableProfile profile = new ChoosableProfile(1, 1, ChoosableProfile.Mode.TRAPEZOID);
        PositionServo<Angle> servo = new PositionServo<>(
                name,
                turningVelocityServo,
                turningEncoder,
                1,
                turningController2,
                profile,
                Angle.instance);
        servo.reset();
        servo.setPosition(1);
        assertEquals(0, turningMotor.output, 0.001);
        assertEquals(0.5, servo.getSetpoint().x(), kDelta);
        assertEquals(1.0, servo.getSetpoint().v(), kDelta);
        assertEquals(1.0, turningMotor.velocity, kDelta);

        Experiments.instance.testOverride(Experiment.UseClosedLoopVelocity, false);

        servo.setPosition(1);
        assertEquals(1.0, turningMotor.output, kDelta);
        assertEquals(1.0, turningMotor.velocity, kDelta);

        servo.stop();
        assertEquals(0, turningMotor.output, kDelta);
        assertEquals(1.0, turningMotor.velocity, kDelta);
    }
}
