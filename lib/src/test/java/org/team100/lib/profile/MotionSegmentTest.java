package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

// passes uncommented
// import com.acmerobotics.roadrunner.profile.MotionSegment;
// import com.acmerobotics.roadrunner.profile.MotionState;

class MotionSegmentTest {
    private static final double kDelta = 0.001;

    @Test
    void testBasic() {
        MotionSegment s = new MotionSegment(new MotionState(0, 1, 0, 0), 1);
        MotionState s1 = s.get(0.5);
        assertEquals(0.5, s1.getX(), kDelta);
        assertEquals(1, s1.getV(), kDelta);
    }
}
