package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Identity;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinematics.SwerveDriveKinematicsFactory;
import org.team100.lib.swerve.SwerveKinematicLimits;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

class TrajectoryTimeSamplerTest {
    private static final double kDelta = 0.001;

    private static final double kMaxVelM_S = 4;
    private static final double kMaxAccelM_S_S = 2;

    @Test
    void testSample() {
        SwerveDriveKinematics m_kinematics = SwerveDriveKinematicsFactory.get(Identity.BLANK);
        SwerveKinematicLimits limits = new SwerveKinematicLimits(4, 2, 2, 10, 7);
        TrajectoryPlanner planner = new TrajectoryPlanner(m_kinematics, limits);
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

        List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        Trajectory100 trajectory = planner
                .generateTrajectory(
                        false,
                        waypointsM,
                        headings,
                        constraints,
                        startVelocity,
                        endVelocity,
                        kMaxVelM_S,
                        kMaxAccelM_S_S);

        TrajectoryTimeSampler sampler = new TrajectoryTimeSampler(trajectory);
        assertEquals(0, sampler.getStartS(), kDelta);
        assertEquals(1.414, sampler.getEndS(), kDelta);
        TrajectorySamplePoint sample = sampler.sample(0).get();
        assertEquals(0, sample.state().state().getPose().getX(), kDelta);
        sample = sampler.sample(1).get();
        assertEquals(0.828, sample.state().state().getPose().getX(), kDelta);
        sample = sampler.sample(2).get();
        assertEquals(1, sample.state().state().getPose().getX(), kDelta);

        assertThrows(IllegalArgumentException.class,
                () -> sampler.sample(Double.NaN));

    }

}
