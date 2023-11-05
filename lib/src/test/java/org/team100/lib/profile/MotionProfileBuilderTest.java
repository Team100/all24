package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

// passes uncommented
// import com.acmerobotics.roadrunner.profile.MotionProfile;
// import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
// import com.acmerobotics.roadrunner.profile.MotionState;

import org.junit.jupiter.api.Test;

class MotionProfileBuilderTest {
    private static final double kDelta = 0.001;

    @Test
    void testBasic() {
        MotionState s = new MotionState(0, 1, 0, 0);
        MotionProfileBuilder b = new MotionProfileBuilder(s);
        b.appendJerkControl(1, 1);
        
        MotionProfile p = b.build();
        assertEquals(1, p.duration(), kDelta);

        MotionState s0 = p.get(0);
        assertEquals(1, s0.getV(), kDelta);

        MotionProfile p1 = p.plus(p);
        assertEquals(2, p1.duration(), kDelta);

        MotionState s1 = p.get(1);
        assertEquals(1.5, s1.getV(), kDelta);

    }
    
}
