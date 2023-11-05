package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Translation2d;

class TimedStateTest {
    private static final double kTestEpsilon = 1e-12;

    @Test
    void test() {
        // At (0,0,0), t=0, v=0, acceleration=1
        TimedPose start_state = new TimedPose(
                new Pose2dWithMotion(
                        GeometryUtil.fromTranslation(new Translation2d(0.0, 0.0)), 0.0),
                0.0, 0.0, 1.0);

        // At (.5,0,0), t=1, v=1, acceleration=0
        TimedPose end_state = new TimedPose(
                new Pose2dWithMotion(
                        GeometryUtil.fromTranslation(new Translation2d(0.5, 0.0)), 0.0),
                1.0, 1.0, 0.0);

        TimedPose i0 = start_state.interpolate2(end_state, 0.0);
        assertEquals(start_state, i0, String.format("%s %s", start_state, i0));
        assertEquals(end_state, start_state.interpolate2(end_state, 1.0));
        assertEquals(end_state, end_state.interpolate2(start_state, 0.0));
        // System.out.println(end_state.interpolate2(start_state, 1.0));
        assertEquals(start_state, end_state.interpolate2(start_state, 1.0));

        TimedPose intermediate_state = start_state.interpolate2(end_state, 0.5);
        assertEquals(0.5, intermediate_state.getTimeS(), kTestEpsilon);
        assertEquals(start_state.acceleration(), intermediate_state.acceleration(), kTestEpsilon);
        assertEquals(0.5, intermediate_state.velocityM_S(), kTestEpsilon);
        assertEquals(0.125, intermediate_state.state().getTranslation().getX(), kTestEpsilon);
    }
}
