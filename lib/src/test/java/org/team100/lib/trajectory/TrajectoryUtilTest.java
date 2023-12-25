package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.path.Path100;
import org.team100.lib.path.PathSamplePoint;
import org.team100.lib.spline.HolonomicSpline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class TrajectoryUtilTest {
    @Test
    void testEmpty() {
        List<HolonomicSpline> splines = new ArrayList<>();
        double maxDx = 0.1;
        double maxDy = 0.1;
        double maxDTheta = 0.1;
        Path100 path = TrajectoryUtil100.trajectoryFromSplines(splines, maxDx, maxDy, maxDTheta);
        assertEquals(0, path.length(), 0.001);
    }

    @Test
    void testSimple() {
        // spline is in the x direction, no curvature.
        HolonomicSpline spline = new HolonomicSpline(
            new Pose2d(),
                new Pose2d(1, 0, new Rotation2d()),
                new Rotation2d(), new Rotation2d()) {

            @Override
            public Translation2d getPoint(double t) {
                return new Translation2d(t, 0);
            }

            @Override
            public Rotation2d getHeading(double t) {
                return GeometryUtil.kRotationZero;
            }

            @Override
            public Optional<Rotation2d> getCourse(double t) {
                return Optional.of(GeometryUtil.kRotationZero);
            }

            @Override
            public double getDHeading(double t) {
                return 0;
            }

            @Override
            public double getCurvature(double t) {
                return 0;
            }

            @Override
            public double getDCurvature(double t) {
                return 0;
            }

            @Override
            public double getVelocity(double t) {
                return 1;
            }
        };
        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(spline);
        double maxDx = 0.1;
        double maxDy = 0.1;
        double maxDTheta = 0.1;
        Path100 path = TrajectoryUtil100.trajectoryFromSplines(splines, maxDx, maxDy, maxDTheta);
        assertEquals(2, path.length(), 0.001);
        {
            PathSamplePoint sample = path.getInterpolated(0);
            Pose2dWithMotion pose = sample.state();
            Pose2d pose2d = pose.getPose();
            assertEquals(0, pose2d.getX(), 0.001);
            assertEquals(0, pose2d.getY(), 0.001);
            assertEquals(0, pose2d.getRotation().getRadians(), 0.001);
        }
        {
            PathSamplePoint sample = path.getInterpolated(0.5);
            Pose2dWithMotion pose = sample.state();
            Pose2d pose2d = pose.getPose();
            assertEquals(0.5, pose2d.getX(), 0.001);
            assertEquals(0, pose2d.getY(), 0.001);
            assertEquals(0, pose2d.getRotation().getRadians(), 0.001);
        }
        {
            PathSamplePoint sample = path.getInterpolated(1);
            Pose2dWithMotion pose = sample.state();
            Pose2d pose2d = pose.getPose();
            assertEquals(1.0, pose2d.getX(), 0.001);
            assertEquals(0, pose2d.getY(), 0.001);
            assertEquals(0, pose2d.getRotation().getRadians(), 0.001);
        }
    }
}
