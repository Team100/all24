package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Random;
import java.util.TreeMap;
import java.util.function.Function;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.TestLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DataLogManager;

class SwerveDrivePoseEstimator100Test {
    private static final double kDelta = 0.001;
    private static final Logger logger = new TestLogger();
    private final SwerveModulePosition p0 = new SwerveModulePosition(0, GeometryUtil.kRotationZero);
    private final SwerveModulePosition[] positionZero = new SwerveModulePosition[] { p0, p0, p0, p0 };
    private final SwerveModulePosition p01 = new SwerveModulePosition(0.1, GeometryUtil.kRotationZero);
    private final SwerveModulePosition[] position01 = new SwerveModulePosition[] { p01, p01, p01, p01 };
    private final Pose2d visionRobotPoseMeters = new Pose2d(1, 0, GeometryUtil.kRotationZero);

    private static void verify(double x, SwerveState state) {
        Pose2d estimate = state.pose();
        assertEquals(x, estimate.getX(), kDelta);
        assertEquals(0, estimate.getY(), kDelta);
        assertEquals(0, estimate.getRotation().getRadians(), kDelta);
    }

    @BeforeEach
    void nolog() {
        DataLogManager.stop();
    }

    @Test
    void outOfOrder() {
        // out of order odometry?
        // no tire slip
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        SwerveDrivePoseEstimator100 poseEstimator = kinodynamics.newPoseEstimator(
                GeometryUtil.kRotationZero,
                positionZero,
                GeometryUtil.kPoseZero,
                0); // zero initial time

        // initial pose = 0
        verify(0, poseEstimator.getEstimatedPosition());

        // pose stays zero when updated at time zero
        // if we try to update zero, there's nothing to compare it to,
        // so we should just ignore this update.
        verify(0, poseEstimator.update(0.0, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(positionZero)));

        // now vision says we're one meter away, so pose goes towards that
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.01, stateStdDevs, visionMeasurementStdDevs);
        verify(0.167, poseEstimator.getEstimatedPosition());

        // if we had added this vision measurement here, it would have pulled the
        // estimate further
        // poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015);
        // verify(0.305, poseEstimator.getEstimatedPosition());

        // wheels haven't moved, so the "odometry opinion" should be zero
        // but it's not, it's applied relative to the vision update, so there's no
        // change.
        verify(0.167, poseEstimator.update(0.02, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(positionZero)));

        // wheels have moved 0.1m in +x, at t=0.04.
        // the "odometry opinion" should be 0.1 since the last odometry estimate was
        // 0, but instead odometry is applied relative to the latest estimate, which
        // was based on vision. so the actual odometry stddev is like *zero*.

        verify(0.267, poseEstimator.update(0.04, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(position01)));

        // here's the delayed update from above, which moves the estimate to 0.305 and
        // then the odometry is applied on top of that, yielding 0.405.
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015, stateStdDevs, visionMeasurementStdDevs);

        verify(0.405, poseEstimator.getEstimatedPosition());

        // wheels are in the same position as the previous iteration,
        verify(0.405, poseEstimator.update(0.06, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(position01)));

        // a little earlier than the previous estimate does nothing.
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.014, stateStdDevs, visionMeasurementStdDevs);
        verify(0.405, poseEstimator.getEstimatedPosition());

        // a little later than the previous estimate works normally.
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.016, stateStdDevs, visionMeasurementStdDevs);
        verify(0.521, poseEstimator.getEstimatedPosition());

        // wheels not moving -> no change,
        verify(0.521, poseEstimator.update(0.08, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(position01)));

    }

    @Test
    void outOfOrderWithSliding() {
        // out of order odometry?
        // use a reasonable max accel.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTestWithSlip(logger);
        Experiments.instance.testOverride(Experiment.SlipperyTires, true);
        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        SwerveDrivePoseEstimator100 poseEstimator = kinodynamics.newPoseEstimator(
                GeometryUtil.kRotationZero,
                positionZero,
                GeometryUtil.kPoseZero,
                0); // zero initial time

        // initial pose = 0
        verify(0, poseEstimator.getEstimatedPosition());

        // pose stays zero when updated at time zero
        // if we try to update zero, there's nothing to compare it to,
        // so we should just ignore this update.
        verify(0, poseEstimator.update(0.0, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(positionZero)));

        // now vision says we're one meter away, so pose goes towards that
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.01, stateStdDevs, visionMeasurementStdDevs);
        verify(0.167, poseEstimator.getEstimatedPosition());

        // if we had added this vision measurement here, it would have pulled the
        // estimate further
        // poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015);
        // verify(0.305, poseEstimator.getEstimatedPosition());

        // wheels haven't moved, so the "odometry opinion" should be zero
        // but it's not, it's applied relative to the vision update, so there's no
        // change.
        verify(0.167, poseEstimator.update(0.02, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(positionZero)));

        // wheels have moved 0.1m in +x, at t=0.04.
        // but the velocity estimate says we're sliding, so it's a bit more than 0.1
        // note the sliding rate is limited here.
        verify(0.267, poseEstimator.update(0.04, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(position01)));

        // here's the delayed update from above, which replays the history
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015, stateStdDevs, visionMeasurementStdDevs);

        verify(0.405, poseEstimator.getEstimatedPosition());

        // wheels are in the same position as the previous iteration,
        // but we've moved since then so we must be sliding.
        verify(0.501, poseEstimator.update(0.06, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(position01)));

        // a little earlier than the previous estimate does nothing.
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.014, stateStdDevs, visionMeasurementStdDevs);
        verify(0.501, poseEstimator.getEstimatedPosition());

        // a little later than the previous estimate works normally.
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.016, stateStdDevs, visionMeasurementStdDevs);
        verify(0.617, poseEstimator.getEstimatedPosition());

        // wheels not moving -> no change,
        // except we are still sliding.
        verify(0.711, poseEstimator.update(0.08, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(position01)));

        verify(0.932, poseEstimator.update(1, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(position01)));
    }

    @Test
    void minorWeirdness() {
        // weirdness with out-of-order vision updates
        // no wheel slip
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        SwerveDrivePoseEstimator100 poseEstimator = kinodynamics.newPoseEstimator(
                GeometryUtil.kRotationZero,
                positionZero,
                GeometryUtil.kPoseZero,
                0); // zero initial time

        // initial pose = 0
        verify(0, poseEstimator.getEstimatedPosition());

        // pose stays zero when updated at time zero
        // if we try to update zero, there's nothing to compare it to,
        // so we should just ignore this update.
        verify(0, poseEstimator.update(0.0, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(positionZero)));

        // now vision says we're one meter away, so pose goes towards that
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.01, stateStdDevs, visionMeasurementStdDevs);
        verify(0.167, poseEstimator.getEstimatedPosition());

        // if we had added this vision measurement here, it would have pulled the
        // estimate further
        // poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015);
        // verify(0.305, poseEstimator.getEstimatedPosition());

        // wheels haven't moved, so the "odometry opinion" should be zero
        // but it's not, it's applied relative to the vision update, so there's no
        // change.
        verify(0.167, poseEstimator.update(0.02, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(positionZero)));

        // wheels have moved 0.1m in +x, at t=0.04.
        // the "odometry opinion" should be 0.1 since the last odometry estimate was
        // 0, but instead odometry is applied relative to the latest estimate, which
        // was based on vision. so the actual odometry stddev is like *zero*.

        verify(0.267, poseEstimator.update(0.04, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(position01)));

        // here's the delayed update from above, which moves the estimate to 0.305 and
        // then the odometry is applied on top of that, yielding 0.405.
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015, stateStdDevs, visionMeasurementStdDevs);

        verify(0.405, poseEstimator.getEstimatedPosition());

        // wheels are in the same position as the previous iteration
        verify(0.405, poseEstimator.update(0.06, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(position01)));

        // a little earlier than the previous estimate does nothing.
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.014, stateStdDevs, visionMeasurementStdDevs);
        verify(0.405, poseEstimator.getEstimatedPosition());

        // a little later than the previous estimate works normally.
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.016, stateStdDevs, visionMeasurementStdDevs);
        verify(0.521, poseEstimator.getEstimatedPosition());

        // wheels not moving -> no change
        verify(0.521, poseEstimator.update(0.08, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(position01)));

    }

    @Test
    void test0105() {
        // this is the current (post-comp 2024) base case.
        // within a few frames, the estimate converges on the vision input.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        SwerveDrivePoseEstimator100 poseEstimator = kinodynamics.newPoseEstimator(
                GeometryUtil.kRotationZero,
                positionZero,
                GeometryUtil.kPoseZero,
                0); // zero initial time

        verify(0, poseEstimator.getEstimatedPosition());
        verify(0, poseEstimator.update(0, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(positionZero)));

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.02, stateStdDevs, visionMeasurementStdDevs);
        verify(0.167, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.04, stateStdDevs, visionMeasurementStdDevs);
        verify(0.305, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.06, stateStdDevs, visionMeasurementStdDevs);
        verify(0.421, poseEstimator.getEstimatedPosition());
    }

    @Test
    void test0110() {
        // double vision stdev (r) -> slower convergence
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 1.0, 1.0, Double.MAX_VALUE };
        SwerveDrivePoseEstimator100 poseEstimator = kinodynamics.newPoseEstimator(
                GeometryUtil.kRotationZero,
                positionZero,
                GeometryUtil.kPoseZero,
                0); // zero initial time

        verify(0, poseEstimator.getEstimatedPosition());
        verify(0, poseEstimator.update(0, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(positionZero)));

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.02, stateStdDevs, visionMeasurementStdDevs);
        verify(0.091, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.04, stateStdDevs, visionMeasurementStdDevs);
        verify(0.173, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.06, stateStdDevs, visionMeasurementStdDevs);
        verify(0.249, poseEstimator.getEstimatedPosition());
    }

    @Test
    void test00505() {
        // half odo stdev (q) -> slower convergence
        // the K is q/(q+qr) so it's q compared to r that matters.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        double[] stateStdDevs = new double[] { 0.05, 0.05, 0.05 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        SwerveDrivePoseEstimator100 poseEstimator = kinodynamics.newPoseEstimator(
                GeometryUtil.kRotationZero,
                positionZero,
                GeometryUtil.kPoseZero,
                0); // zero initial time

        verify(0, poseEstimator.getEstimatedPosition());
        verify(0, poseEstimator.update(0, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(positionZero)));

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.02, stateStdDevs, visionMeasurementStdDevs);
        verify(0.091, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.04, stateStdDevs, visionMeasurementStdDevs);
        verify(0.173, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.06, stateStdDevs, visionMeasurementStdDevs);
        verify(0.249, poseEstimator.getEstimatedPosition());
    }

    @Test
    void reasonable() {
        // stdev that actually make sense
        // actual odometry error is very low
        // measured camera error is something under 10 cm
        // these yield much slower convergence, maybe too slow? try and see.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest(logger);
        double[] stateStdDevs = new double[] { 0.001, 0.001, 0.01 };
        double[] visionMeasurementStdDevs = new double[] { 0.1, 0.1, Double.MAX_VALUE };
        SwerveDrivePoseEstimator100 poseEstimator = kinodynamics.newPoseEstimator(
                GeometryUtil.kRotationZero,
                positionZero,
                GeometryUtil.kPoseZero,
                0); // zero initial time
        verify(0, poseEstimator.getEstimatedPosition());
        verify(0, poseEstimator.update(0, GeometryUtil.kRotationZero,
                new SwerveDriveWheelPositions(positionZero)));

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.02, stateStdDevs, visionMeasurementStdDevs);
        verify(0.010, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.04, stateStdDevs, visionMeasurementStdDevs);
        verify(0.020, poseEstimator.getEstimatedPosition());

        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.06, stateStdDevs, visionMeasurementStdDevs);
        verify(0.029, poseEstimator.getEstimatedPosition());
    }

    ////////////////////////////////////////
    //
    // tests below are from WPILib
    //
    //

    @Test
    void testAccuracyFacingTrajectory() {
        // no wheel slip
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forWPITest(logger);

        var fl = new SwerveModulePosition();
        var fr = new SwerveModulePosition();
        var bl = new SwerveModulePosition();
        var br = new SwerveModulePosition();

        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, 0.5 };
        var estimator = new SwerveDrivePoseEstimator100(
                logger,
                kinodynamics,
                new Rotation2d(),
                new SwerveModulePosition[] { fl, fr, bl, br },
                new Pose2d(),
                0); // zero initial time

        var trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                        new Pose2d(3, 0, Rotation2d.fromDegrees(-90)),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(135)),
                        new Pose2d(-3, 0, Rotation2d.fromDegrees(-90)),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(45))),
                new TrajectoryConfig(2, 2));

        testFollowTrajectory(
                kinodynamics.getKinematics(),
                estimator,
                stateStdDevs,
                visionMeasurementStdDevs,
                trajectory,
                state -> new ChassisSpeeds(
                        state.velocityMetersPerSecond,
                        0,
                        state.velocityMetersPerSecond * state.curvatureRadPerMeter),
                state -> state.poseMeters,
                trajectory.getInitialPose(),
                new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                0.02,
                0.1,
                0.25,
                true);

    }

    @Test
    void testBadInitialPose() {
        // no wheel slip
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forWPITest(logger);

        var fl = new SwerveModulePosition();
        var fr = new SwerveModulePosition();
        var bl = new SwerveModulePosition();
        var br = new SwerveModulePosition();

        // estimator initial pose is at (-1,-1) rotated 60 degrees to the right.

        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        // this doesn't work if you disregard vision theta
        // because the whole point is to fix the very-wrong offset.
        // VecBuilder.fill(0.9, 0.9, Double.MAX_VALUE));
        double[] visionMeasurementStdDevs = new double[] { 0.9, 0.9, 0.9 };
        var estimator = new SwerveDrivePoseEstimator100(
                logger,
                kinodynamics,
                new Rotation2d(),
                new SwerveModulePosition[] { fl, fr, bl, br },
                new Pose2d(-1, -1, Rotation2d.fromRadians(-1)),
                0); // zero initial time

        var trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                        new Pose2d(3, 0, Rotation2d.fromDegrees(-90)),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(135)),
                        new Pose2d(-3, 0, Rotation2d.fromDegrees(-90)),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(45))),
                new TrajectoryConfig(2, 2));

        for (int offset_direction_degs = 0; offset_direction_degs < 360; offset_direction_degs += 45) {
            for (int offset_heading_degs = 0; offset_heading_degs < 360; offset_heading_degs += 45) {
                var pose_offset = Rotation2d.fromDegrees(offset_direction_degs);
                var heading_offset = Rotation2d.fromDegrees(offset_heading_degs);

                // actual initial pose is offset from the initial trajectory pose

                var initial_pose = trajectory
                        .getInitialPose()
                        .plus(
                                new Transform2d(
                                        new Translation2d(pose_offset.getCos(), pose_offset.getSin()),
                                        heading_offset));

                testFollowTrajectory(
                        kinodynamics.getKinematics(),
                        estimator,
                        stateStdDevs,
                        visionMeasurementStdDevs,
                        trajectory,
                        state -> new ChassisSpeeds(
                                state.velocityMetersPerSecond,
                                0, // vy = 0 -> drive like a tank
                                state.velocityMetersPerSecond * state.curvatureRadPerMeter),
                        state -> state.poseMeters,
                        initial_pose,
                        new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                        0.02,
                        0.1,
                        1.0,
                        false);

            }
        }
    }

    void testFollowTrajectory(
            final SwerveDriveKinematics100 kinematics,
            final SwerveDrivePoseEstimator100 estimator,
            final double[] stateStdDevs,
            final double[] visionMeasurementStdDevs,
            final Trajectory trajectory,
            final Function<Trajectory.State, ChassisSpeeds> chassisSpeedsGenerator,
            final Function<Trajectory.State, Pose2d> visionMeasurementGenerator,
            final Pose2d startingPose,
            final Pose2d endingPose,
            final double dt,
            final double visionUpdateRate,
            final double visionUpdateDelay,
            final boolean checkError) {
        SwerveModulePosition[] positions = {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };

        // new starting pose here, so we don't actually use the earlier initial pose

        estimator.resetPosition(
                new Rotation2d(),
                new SwerveDriveWheelPositions(positions),
                startingPose,
                0); // zero initial time

        var rand = new Random(3538);

        double t = 0.0;

        final TreeMap<Double, Pose2d> visionUpdateQueue = new TreeMap<>();

        double maxError = Double.NEGATIVE_INFINITY;
        double errorSum = 0;
        while (t <= trajectory.getTotalTimeSeconds()) {
            State groundTruthState = trajectory.sample(t);

            // We are due for a new vision measurement if it's been `visionUpdateRate`
            // seconds since the last vision measurement
            if (visionUpdateQueue.isEmpty() || visionUpdateQueue.lastKey() + visionUpdateRate < t) {
                Pose2d newVisionPose = visionMeasurementGenerator
                        .apply(groundTruthState)
                        .plus(
                                new Transform2d(
                                        new Translation2d(rand.nextGaussian() * 0.1, rand.nextGaussian() * 0.1),
                                        new Rotation2d(rand.nextGaussian() * 0.05)));

                visionUpdateQueue.put(t, newVisionPose);
            }

            // We should apply the oldest vision measurement if it has been
            // `visionUpdateDelay` seconds since it was measured
            if (!visionUpdateQueue.isEmpty() && visionUpdateQueue.firstKey() + visionUpdateDelay < t) {
                var visionEntry = visionUpdateQueue.pollFirstEntry();
                estimator.addVisionMeasurement(visionEntry.getValue(), visionEntry.getKey(), stateStdDevs,
                        visionMeasurementStdDevs);
            }

            var chassisSpeeds = chassisSpeedsGenerator.apply(groundTruthState);

            var moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

            for (int i = 0; i < moduleStates.length; i++) {
                positions[i].distanceMeters += moduleStates[i].speedMetersPerSecond * (1 - rand.nextGaussian() * 0.05)
                        * dt;
                positions[i].angle = moduleStates[i].angle.plus(new Rotation2d(rand.nextGaussian() * 0.005));
            }

            SwerveState xHat = estimator.update(
                    t,
                    groundTruthState.poseMeters
                            .getRotation()
                            .plus(new Rotation2d(rand.nextGaussian() * 0.05))
                            .minus(trajectory.getInitialPose().getRotation()),
                    new SwerveDriveWheelPositions(positions));

            double error = groundTruthState.poseMeters.getTranslation().getDistance(xHat.pose().getTranslation());
            if (error > maxError) {
                maxError = error;
            }
            errorSum += error;

            t += dt;

        }

        assertEquals(
                endingPose.getX(), estimator.getEstimatedPosition().pose().getX(), 0.08, "Incorrect Final X");
        assertEquals(
                endingPose.getY(), estimator.getEstimatedPosition().pose().getY(), 0.08, "Incorrect Final Y");
        assertEquals(
                endingPose.getRotation().getRadians(),
                estimator.getEstimatedPosition().pose().getRotation().getRadians(),
                0.15,
                "Incorrect Final Theta");

        if (checkError) {
            assertEquals(
                    0.0, errorSum / (trajectory.getTotalTimeSeconds() / dt), 0.07, "Incorrect mean error");
            assertEquals(0.0, maxError, 0.2, "Incorrect max error");
        }
    }

    @Test
    void testSimultaneousVisionMeasurements() {
        // This tests for multiple vision measurements appled at the same time. The
        // expected behavior
        // is that all measurements affect the estimated pose. The alternative result is
        // that only one
        // vision measurement affects the outcome. If that were the case, after 1000
        // measurements, the
        // estimated pose would converge to that measurement.

        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forWPITest(logger);

        var fl = new SwerveModulePosition();
        var fr = new SwerveModulePosition();
        var bl = new SwerveModulePosition();
        var br = new SwerveModulePosition();

        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.9, 0.9, 0.9 };
        var estimator = new SwerveDrivePoseEstimator100(
                logger,
                kinodynamics,
                new Rotation2d(),
                new SwerveModulePosition[] { fl, fr, bl, br },
                new Pose2d(1, 2, Rotation2d.fromDegrees(270)),
                0); // zero initial time

        estimator.update(0,
                new Rotation2d(), new SwerveDriveWheelPositions(
                        new SwerveModulePosition[] { fl, fr, bl, br }));

        var visionMeasurements = new Pose2d[] {
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(3, 1, Rotation2d.fromDegrees(90)),
                new Pose2d(2, 4, Rotation2d.fromRadians(180)),
        };

        for (int i = 0; i < 1000; i++) {
            for (var measurement : visionMeasurements) {
                estimator.addVisionMeasurement(measurement, 0, stateStdDevs, visionMeasurementStdDevs);
            }
        }

        for (var measurement : visionMeasurements) {
            var errorLog = "Estimator converged to one vision measurement: "
                    + estimator.getEstimatedPosition().toString()
                    + " -> "
                    + measurement;

            var dx = Math.abs(measurement.getX() - estimator.getEstimatedPosition().pose().getX());
            var dy = Math.abs(measurement.getY() - estimator.getEstimatedPosition().pose().getY());
            var dtheta = Math.abs(
                    measurement.getRotation().getDegrees()
                            - estimator.getEstimatedPosition().pose().getRotation().getDegrees());

            assertTrue(dx > 0.08 || dy > 0.08 || dtheta > 0.08, errorLog);
        }
    }

    @Test
    void testDiscardsOldVisionMeasurements() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forWPITest(logger);
        var estimator = new SwerveDrivePoseEstimator100(
                logger,
                kinodynamics,
                new Rotation2d(),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                },
                new Pose2d(),
                0); // zero initial time

        double time = 0;

        // Add enough measurements to fill up the buffer
        for (; time < 4; time += 0.02) {
            estimator.update(
                    time,
                    new Rotation2d(),
                    new SwerveDriveWheelPositions(
                            new SwerveModulePosition[] {
                                    new SwerveModulePosition(),
                                    new SwerveModulePosition(),
                                    new SwerveModulePosition(),
                                    new SwerveModulePosition()
                            }));
        }

        var odometryPose = estimator.getEstimatedPosition();

        // Apply a vision measurement made 3 seconds ago
        // This test passes if this does not cause a ConcurrentModificationException.
        estimator.addVisionMeasurement(
                new Pose2d(new Translation2d(10, 10), new Rotation2d(0.1)),
                1,
                new double[] { 0.1, 0.1, 0.1 },
                new double[] { 0.1, 0.1, 0.1 });

        assertEquals(odometryPose.pose().getX(), estimator.getEstimatedPosition().pose().getX(), "Incorrect Final X");
        assertEquals(odometryPose.pose().getY(), estimator.getEstimatedPosition().pose().getY(), "Incorrect Final Y");
        assertEquals(
                odometryPose.pose().getRotation().getRadians(),
                estimator.getEstimatedPosition().pose().getRotation().getRadians(),
                "Incorrect Final Theta");
    }
}