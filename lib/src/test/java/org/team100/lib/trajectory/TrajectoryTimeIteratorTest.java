package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

class TrajectoryTimeIteratorTest {
    private static final double kDelta = 0.001;
    private static final double kMaxVelM_S = 4;
    private static final double kMaxAccelM_S_S = 2;
    private static final SupplierLogger2 logger = new TestLogger().getSupplierLogger();

    @Test
    void testPreviewAndAdvance() {

        SwerveKinodynamics limits = SwerveKinodynamicsFactory.get(logger);
        Pose2d start = GeometryUtil.kPoseZero;
        double startVelocity = 0;
        Pose2d end = start.plus(new Transform2d(1, 0, GeometryUtil.kRotationZero));
        double endVelocity = 0;

        Translation2d currentTranslation = start.getTranslation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        List<Pose2d> waypointsM = List.of(
                new Pose2d(currentTranslation, angleToGoal),
                new Pose2d(goalTranslation, angleToGoal));

        List<Rotation2d> headings = List.of(
                start.getRotation(),
                end.getRotation());

        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).forTest();

        Trajectory100 trajectory = TrajectoryPlanner.generateTrajectory(
                waypointsM,
                headings,
                constraints,
                startVelocity,
                endVelocity,
                kMaxVelM_S,
                kMaxAccelM_S_S);

        TrajectoryTimeSampler sampler = new TrajectoryTimeSampler(trajectory);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(sampler);

        TrajectorySamplePoint sample = iter.preview(0).get();
        assertEquals(0, sample.state().state().getPose().getX(), kDelta);
        sample = iter.advance(0).get();
        assertEquals(0, sample.state().state().getPose().getX(), kDelta);

        sample = iter.preview(1).get();
        assertEquals(0.828, sample.state().state().getPose().getX(), kDelta);
        sample = iter.advance(1).get();
        assertEquals(0.828, sample.state().state().getPose().getX(), kDelta);

        sample = iter.preview(1).get();
        assertEquals(1, sample.state().state().getPose().getX(), kDelta);
        sample = iter.advance(1).get();
        assertEquals(1, sample.state().state().getPose().getX(), kDelta);
    }

}
