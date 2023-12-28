package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.turning.MockEncoder100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.MockMotor100;
import org.team100.lib.profile.ChoosableProfile;
import org.team100.lib.units.Angle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
        assertEquals(0.5, servo.getSetpoint().position, kDelta);
        assertEquals(1.0, servo.getSetpoint().velocity, kDelta);
        assertEquals(1.0, turningMotor.velocity, kDelta);

        Experiments.instance.testOverride(Experiment.UseClosedLoopVelocity, false);

        servo.setPosition(1);
        assertEquals(1.0, turningMotor.output, kDelta);
        assertEquals(1.0, turningMotor.velocity, kDelta);

        servo.stop();
        assertEquals(0, turningMotor.output, kDelta);
        assertEquals(1.0, turningMotor.velocity, kDelta);
    }

    /**
     * Duplicates the WPILib profile wrapping logic, and shows that it's wrong.
     */
    @Test
    void testWrapping() {
        double m_maximumInput = 2 * Math.PI;
        double m_minimumInput = 0;
        double errorBound = (m_maximumInput - m_minimumInput) / 2.0;

        // the measurement and setpoint are the same.
        // the state is past 180 degrees but moving negatively, so it's
        // faster to go the "long way" because of the velocity.
        // but the wrapping calculation ignores velocity.
        double measurement = 4;
        TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(4, -1);
        TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

        double goalMinDistance = MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
        double setpointMinDistance = MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound,
                errorBound);

        m_goal.position = goalMinDistance + measurement;
        m_setpoint.position = setpointMinDistance + measurement;

        // this should be zero; trying to slow down and stop
        // is much worse than continuing the long way around.
        assertEquals(2 * Math.PI, m_goal.position, kDelta);
        assertEquals(0, m_goal.velocity, kDelta);
        assertEquals(4, m_setpoint.position, kDelta);
        assertEquals(-1, m_setpoint.velocity, kDelta);
    }



}
