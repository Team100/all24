package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

class SwerveStateTest {
    private static final double kDelta = 0.001;
    @Test
    void testTransform() {
        Pose2d p = new Pose2d(new Translation2d(1,1), new Rotation2d(1));
        Twist2d t = new Twist2d(1,1,1);
        SwerveState s = new SwerveState(p, t);
        assertEquals(1,s.x().x(), kDelta);
    }
    
}
