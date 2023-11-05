package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

// passes uncommented
// import com.acmerobotics.roadrunner.profile.MotionProfile;
// import com.acmerobotics.roadrunner.profile.MotionState;
// import com.acmerobotics.roadrunner.profile.MotionSegment;

import org.junit.jupiter.api.Test;

class MotionProfileTest {
    private static final double kDelta = 0.001;

    @Test
    void testBasic() {
        List<MotionSegment> segments = new ArrayList<MotionSegment>();
        segments.add(new MotionSegment(new MotionState(0, 1, 0, 0), 1));
        MotionProfile p = new MotionProfile(segments);
        assertEquals(1, p.duration(), kDelta);

        MotionState s0 = p.get(0);
        assertEquals(1, s0.getV(), kDelta);

        MotionProfile p1 = p.plus(p);
        assertEquals(2, p1.duration(), kDelta);

        MotionState s1 = p.get(1);
        assertEquals(1, s1.getV(), kDelta);

    }

    @Test
    void testReverse() {
        List<MotionSegment> segments = new ArrayList<MotionSegment>();
        segments.add(new MotionSegment(new MotionState(0, 1, 0, 0), 1));
        MotionProfile p = new MotionProfile(segments);
        MotionProfile b = p.reversed();
        assertEquals(1, b.duration(), kDelta);
    }
}
