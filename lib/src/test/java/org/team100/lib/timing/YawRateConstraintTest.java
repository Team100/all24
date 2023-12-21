package org.team100.lib.timing;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;

class YawRateConstraintTest {
    @Test
    void testSimple() {
        // one radian/s
        YawRateConstraint c = new YawRateConstraint(1);
        Pose2dWithMotion p = new Pose2dWithMotion(new Pose2d(), new Twist2d(1,0,0), 0);
    }
    
}
