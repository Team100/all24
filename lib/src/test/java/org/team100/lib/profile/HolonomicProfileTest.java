package org.team100.lib.profile;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Pose2d;

class HolonomicProfileTest {
    @Test
    void test2d() {
        HolonomicProfile hp = new HolonomicProfile(0.02, 1, 1, 0.01, 1, 1, 0.01);
        SwerveState i = new SwerveState();
        SwerveState g = new SwerveState(new Pose2d(1, 5, GeometryUtil.kRotationZero));
        hp.solve(i, g);
        SwerveState s = i;
        for (double t = 0; t < 10; t += 0.02) {
            s = hp.calculate(s, g);
            System.out.printf("%.2f %.3f %.3f\n", t, s.x().x(), s.y().x());
        }
    }
    @Test
    void test2dWithEntrySpeed() {
        HolonomicProfile hp = new HolonomicProfile(0.02, 1, 1, 0.01, 1, 1, 0.01);
        SwerveState i = new SwerveState(new Pose2d(), new FieldRelativeVelocity(1,0,0));
        SwerveState g = new SwerveState(new Pose2d(0, 1, GeometryUtil.kRotationZero));
        hp.solve(i, g);
        SwerveState s = i;
        for (double t = 0; t < 10; t += 0.02) {
            s = hp.calculate(s, g);
            System.out.printf("%.2f %.3f %.3f\n", t, s.x().x(), s.y().x());
        }
    }
}
