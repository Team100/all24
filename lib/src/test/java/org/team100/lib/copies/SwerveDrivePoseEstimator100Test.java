package org.team100.lib.copies;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.Fixture;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

class SwerveDrivePoseEstimator100Test {
    private static final double kDelta = 0.001;
    private final SwerveModulePosition p0 = new SwerveModulePosition(0, GeometryUtil.kRotationZero);
    private final SwerveModulePosition[] positionZero = new SwerveModulePosition[] { p0, p0, p0, p0 };
    private final SwerveModulePosition p01 = new SwerveModulePosition(0.1, GeometryUtil.kRotationZero);
    private final SwerveModulePosition[] position01 = new SwerveModulePosition[] { p01, p01, p01, p01 };
    private final Pose2d visionRobotPoseMeters = new Pose2d(1, 0, GeometryUtil.kRotationZero);

    private final Fixture fixture = new Fixture();

    private static void verify(double x, Pose2d estimate) {
        assertEquals(x, estimate.getX(), kDelta);
        assertEquals(0, estimate.getY(), kDelta);
        assertEquals(0, estimate.getRotation().getRadians(), kDelta);
    }

    @Test
    void minorWeirdness() {
        // weirdness with out-of-order vision updates
        SwerveDrivePoseEstimator100 poseEstimator = fixture.swerveKinodynamics.newPoseEstimator(
                GeometryUtil.kRotationZero,
                positionZero,
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE));

        // initial pose = 0
        verify(0, poseEstimator.getEstimatedPosition());

        // pose stays zero when updated at time zero
        verify(0, poseEstimator.updateWithTime(0.0, GeometryUtil.kRotationZero, positionZero));

        // now vision says we're one meter away, so pose goes towards that
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.01);
        verify(0.167, poseEstimator.getEstimatedPosition());

        // if we had added this vision measurement here, it would have pulled the
        // estimate further
        // poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015);
        // verify(0.305, poseEstimator.getEstimatedPosition());

        // wheels haven't moved, so the "odometry opinion" should be zero
        // but it's not, it's applied relative to the vision update, so there's no
        // change.
        verify(0.167, poseEstimator.updateWithTime(0.02, GeometryUtil.kRotationZero, positionZero));

        // wheels have moved 0.1m in +x, at t=0.04.
        // the "odometry opinion" should be 0.1 since the last odometry estimate was
        // 0, but instead odometry is applied relative to the latest estimate, which
        // was based on vision. so the actual odometry stddev is like *zero*.
        verify(0.267, poseEstimator.updateWithTime(0.04, GeometryUtil.kRotationZero, position01));

        // here's the delayed update from above, which moves the estimate to 0.305 and
        // then the odometry is applied on top of that, yielding 0.405.
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015);
        verify(0.405, poseEstimator.getEstimatedPosition());

        // wheels are in the same position as the previous iteration
        verify(0.405, poseEstimator.updateWithTime(0.06, GeometryUtil.kRotationZero, position01));

        // a little earlier than the previous estimate does nothing.
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.014);
        verify(0.405, poseEstimator.getEstimatedPosition());

        // a little later than the previous estimate works normally.
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.016);
        verify(0.521, poseEstimator.getEstimatedPosition());

        // wheels not moving -> no change
        verify(0.521, poseEstimator.updateWithTime(0.08, GeometryUtil.kRotationZero, position01));
    }

    @Test
    void test0105() {
        // this is the current (post-comp 2024) base case.
        // within a few frames, the estimate converges on the vision input.
        SwerveDrivePoseEstimator100 poseEstimator = fixture.swerveKinodynamics.newPoseEstimator(
                GeometryUtil.kRotationZero,
                positionZero,
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE)); // 0.1 0.1
        verify(0, poseEstimator.getEstimatedPosition());
        verify(0, poseEstimator.updateWithTime(0, GeometryUtil.kRotationZero, positionZero));

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.02);
        verify(0.167, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.04);
        verify(0.305, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.06);
        verify(0.421, poseEstimator.getEstimatedPosition());
    }

    @Test
    void test0110() {
        // double vision stdev (r) -> slower convergence
        SwerveDrivePoseEstimator100 poseEstimator = fixture.swerveKinodynamics.newPoseEstimator(
                GeometryUtil.kRotationZero,
                positionZero,
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(1.0, 1.0, Double.MAX_VALUE));
        verify(0, poseEstimator.getEstimatedPosition());
        verify(0, poseEstimator.updateWithTime(0, GeometryUtil.kRotationZero, positionZero));

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.02);
        verify(0.091, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.04);
        verify(0.173, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.06);
        verify(0.249, poseEstimator.getEstimatedPosition());
    }

    @Test
    void test00505() {
        // half odo stdev (q) -> slower convergence
        // the K is q/(q+qr) so it's q compared to r that matters.
        SwerveDrivePoseEstimator100 poseEstimator = fixture.swerveKinodynamics.newPoseEstimator(
                GeometryUtil.kRotationZero,
                positionZero,
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.05, 0.05, 0.05),
                VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE));
        verify(0, poseEstimator.getEstimatedPosition());
        verify(0, poseEstimator.updateWithTime(0, GeometryUtil.kRotationZero, positionZero));

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.02);
        verify(0.091, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.04);
        verify(0.173, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.06);
        verify(0.249, poseEstimator.getEstimatedPosition());
    }

}
