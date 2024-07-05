package org.team100.lib.controller;

import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Follow a 254 trajectory using a Ramsete controller.
 * 
 * This seems to have a large tolerance value? Or doesn't care about hitting the
 * endpoint?
 * 
 * This originated in DriveMotionPlanner, which included several
 * controllers.
 */
public class DriveRamseteController implements DriveMotionController {
    private static final double kThetaKp = 5.0; // Units are rad/s per rad of error.
    private static final double kBeta = 2.0; // >0.
    private static final double kZeta = 0.7; // Damping coefficient, [0, 1].
    private static final double kLooperDt = 0.02;

    private final Logger m_logger;

    public DriveRamseteController(Logger parent) {
        m_logger = parent.child(this);
    }

    private TrajectoryTimeIterator m_iter;
    private double mLastTime = Double.POSITIVE_INFINITY;

    @Override
    public void setTrajectory(final TrajectoryTimeIterator trajectory) {
        m_iter = trajectory;
        mLastTime = Double.POSITIVE_INFINITY;
    }

    /**
     * Implements eq. 5.12 from
     * https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
     */
    @Override
    public ChassisSpeeds update(double timestamp, Pose2d measurement, ChassisSpeeds measurementV) {
        if (m_iter == null)
            return null;

        m_logger.logPose2d(Level.TRACE, "current state", () -> measurement);
        if (isDone()) {
            return new ChassisSpeeds();
        }

        Optional<TimedPose> optionalSetpoint = getSetpoint(timestamp);
        if (!optionalSetpoint.isPresent()) {
            return new ChassisSpeeds();
        }
        TimedPose setpoint = optionalSetpoint.get();
        m_logger.logTimedPose(Level.TRACE, "setpoint", () -> setpoint);

        // Convert from current velocity into course.
        Optional<Rotation2d> maybe_field_to_course = Optional.empty();
        // robot-relative course
        Optional<Rotation2d> maybe_robot_to_course = GeometryUtil.getCourse(measurementV);
        if (maybe_robot_to_course.isPresent()) {
            // Course is robot_to_course, we want to be field_to_course.
            // field_to_course = field_to_robot * robot_to_course
            maybe_field_to_course = Optional.of(measurement.getRotation().rotateBy(maybe_robot_to_course.get()));
        }

        // Convert goal into a desired course (in robot frame).
        double goal_linear_velocity = setpoint.velocityM_S();
        double goal_angular_velocity = goal_linear_velocity * setpoint.state().getCurvature();
        Optional<Rotation2d> maybe_field_to_goal = setpoint.state().getCourse();

        // Deal with lack of course data by always being optimistic.
        if (maybe_field_to_course.isEmpty()) {
            maybe_field_to_course = maybe_field_to_goal;
        }
        if (maybe_field_to_goal.isEmpty()) {
            maybe_field_to_goal = maybe_field_to_course;
        }
        if (maybe_field_to_goal.isEmpty() && maybe_field_to_course.isEmpty()) {
            // Course doesn't matter.
            maybe_field_to_course = Optional.of(GeometryUtil.kRotationZero);
            maybe_field_to_goal = Optional.of(GeometryUtil.kRotationZero);
        }
        Rotation2d field_to_course = maybe_field_to_course.get();

        Rotation2d robot_to_course = measurement.getRotation().unaryMinus().rotateBy(field_to_course);

        // Convert goal course to be relative to current course.
        // course_to_goal = course_to_field * field_to_goal
        Rotation2d course_to_goal = field_to_course.unaryMinus().rotateBy(maybe_field_to_goal.get());

        Twist2d mErrorTwist = DriveMotionControllerUtil.getErrorTwist(measurement, setpoint);
        m_logger.logTwist2d(Level.TRACE, "error", () -> mErrorTwist);

        // Rotate error to be aligned to current course.
        // Error is in robot (heading) frame. Need to rotate it to be in course frame.
        // course_to_error = robot_to_course.inverse() * robot_to_error
        Translation2d linear_error_course_relative = GeometryUtil.fromRotation(
                robot_to_course).exp(mErrorTwist).getTranslation();

        // Compute time-varying gain parameter.
        final double k = 2.0 * kZeta * Math.sqrt(
                kBeta * goal_linear_velocity * goal_linear_velocity + goal_angular_velocity * goal_angular_velocity);

        // Compute error components.
        final double angle_error_rads = course_to_goal.getRadians();
        final double sin_x_over_x = Math100.epsilonEquals(angle_error_rads, 0.0, 1E-2) ? 1.0
                : course_to_goal.getSin() / angle_error_rads;
        double adjusted_linear_velocity = goal_linear_velocity * course_to_goal.getCos()
                + k * linear_error_course_relative.getX();
        double adjusted_angular_velocity = goal_angular_velocity + k * angle_error_rads
                + goal_linear_velocity * kBeta * sin_x_over_x * linear_error_course_relative.getY();

        double heading_rate = goal_linear_velocity * setpoint.state().getHeadingRate()
                + kThetaKp * mErrorTwist.dtheta;

        // Create a course-relative Twist2d.
        Twist2d adjusted_course_relative_velocity = new Twist2d(adjusted_linear_velocity, 0.0,
                adjusted_angular_velocity - heading_rate);

        // See where that takes us in one dt.
        Pose2d adjusted_course_to_goal = GeometryUtil.kPoseZero
                .exp(GeometryUtil.scale(adjusted_course_relative_velocity, kLooperDt));

        // Now rotate to be robot-relative.
        // robot_to_goal = robot_to_course * course_to_goal
        Translation2d adjusted_robot_to_goal = GeometryUtil
                .transformBy(GeometryUtil.fromRotation(robot_to_course), adjusted_course_to_goal).getTranslation()
                .times(1.0 / kLooperDt);

        return new ChassisSpeeds(
                adjusted_robot_to_goal.getX(),
                adjusted_robot_to_goal.getY(),
                heading_rate);
    }

    Optional<TimedPose> getSetpoint(final double timestamp) {
        if (!Double.isFinite(mLastTime))
            mLastTime = timestamp;
        final double mDt = timestamp - mLastTime;
        mLastTime = timestamp;

        Optional<TrajectorySamplePoint> sample_point = m_iter.advance(mDt);
        if (!sample_point.isPresent()) {
            Util.warn("Failed to advance");
            return Optional.empty();
        }

        m_logger.logTrajectorySamplePoint(Level.TRACE, "sample point", sample_point::get);
        return Optional.of(sample_point.get().state());
    }

    @Override
    public boolean isDone() {
        return m_iter != null && m_iter.isDone();
    }

    @Override
    public String getGlassName() {
        return "DriveRamseteController";
    }
}
