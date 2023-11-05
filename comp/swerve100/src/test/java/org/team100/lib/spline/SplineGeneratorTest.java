package org.team100.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

class SplineGeneratorTest {
    @Test
    void test() {
        Pose2d p1 = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        Pose2d p2 = new Pose2d(new Translation2d(15, 10), new Rotation2d(1, -5));
        PoseSpline s = new QuinticHermitePoseSplineNonholonomic(p1, p2);

        List<Pose2dWithMotion> samples = SplineGenerator.parameterizeSpline(s);

        double arclength = 0;
        Pose2dWithMotion cur_pose = samples.get(0);
        for (Pose2dWithMotion sample : samples) {
            final Twist2d twist = GeometryUtil.slog(
                    GeometryUtil.transformBy(
                            GeometryUtil.inverse(cur_pose.getPose()), sample.getPose()));
            arclength += twist.dx;
            cur_pose = sample;
        }

        assertEquals(15.0, cur_pose.getTranslation().getX(), 0.001);
        assertEquals(10.0, cur_pose.getTranslation().getY(), 0.001);
        assertEquals(-78.690, cur_pose.getRotation().getDegrees(), 0.001);
        assertEquals(23.218, arclength, 0.001);
    }
}
