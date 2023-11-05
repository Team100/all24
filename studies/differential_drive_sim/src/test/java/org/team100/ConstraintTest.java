package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * This test explores one of the causes of TrajectoryGenerationException. If you
 * see a message like this:
 * 
 * Unhandled exception:
 * edu.wpi.first.math.trajectory.TrajectoryParameterizer$TrajectoryGenerationException:
 * The constraint's min acceleration was greater than its max acceleration.
 * Offending Constraint:
 * edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint
 * 
 * The problem may be that the TrajectoryConfig max speed is higher than the
 * drivetrain feedforward can supply.
 */

public class ConstraintTest {
    @Test
    void testConstraintBad() {
        double ks = 0.2;
        int kv = 4;
        double ka = 0.5;
        SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        double maxVoltage = 7;

        // the requested wheel speed is too high.
        // max speed is v/kv = 1.75m/s
        // so the "max accel" math is messed up.

        double maxWheelSpeed = 3;
        double minWheelSpeed = -1.2;

        double maxWheelAcceleration = m_feedforward.maxAchievableAcceleration(maxVoltage, maxWheelSpeed);
        double minWheelAcceleration = m_feedforward.minAchievableAcceleration(maxVoltage, minWheelSpeed);

        // note the max is less than the min

        assertEquals(-10.3, maxWheelAcceleration, 0.1);
        assertEquals(-4.0, minWheelAcceleration, 0.1);

        assertFalse(minWheelAcceleration < maxWheelAcceleration);
    }

    @Test
    void testConstraint() {

        double ks = 0.2;
        int kv = 4;
        double ka = 0.5;
        SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        double maxVoltage = 7;

        // these speeds are slow enough to be feasible.

        double maxWheelSpeed = 0.6;
        double minWheelSpeed = -0.3;

        double maxWheelAcceleration = m_feedforward.maxAchievableAcceleration(maxVoltage, maxWheelSpeed);
        double minWheelAcceleration = m_feedforward.minAchievableAcceleration(maxVoltage, minWheelSpeed);
        assertEquals(8.8, maxWheelAcceleration, 0.1);
        assertEquals(-11.2, minWheelAcceleration, 0.1);

        assertTrue(minWheelAcceleration < maxWheelAcceleration);
    }

}
