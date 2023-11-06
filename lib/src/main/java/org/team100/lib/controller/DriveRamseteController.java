package org.team100.lib.controller;

import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.planners.DriveMotionPlanner;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.util.MathUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This originated in DriveMotionPlanner, which included several
 * controllers.
 */
public class DriveRamseteController {

    public void reset() {

    }

    public ChassisSpeeds updateRamsete(
            TimedPose fieldToGoal,
            Pose2d fieldToRobot,
            Twist2d currentVelocity,
            Pose2d mError) {
        // Implements eqn. 5.12 from
        // https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
        final double kBeta = 2.0; // >0.
        final double kZeta = 0.7; // Damping coefficient, [0, 1].

        // Convert from current velocity into course.
        Optional<Rotation2d> maybe_field_to_course = Optional.empty();
        Optional<Rotation2d> maybe_robot_to_course = GeometryUtil.getCourse(currentVelocity);
        if (maybe_robot_to_course.isPresent()) {
            // Course is robot_to_course, we want to be field_to_course.
            // field_to_course = field_to_robot * robot_to_course
            maybe_field_to_course = Optional.of(fieldToRobot.getRotation().rotateBy(maybe_robot_to_course.get()));
        }

        // Convert goal into a desired course (in robot frame).
        double goal_linear_velocity = fieldToGoal.velocityM_S();
        double goal_angular_velocity = goal_linear_velocity * fieldToGoal.state().getCurvature();
        Optional<Rotation2d> maybe_field_to_goal = fieldToGoal.state().getCourse();

        // Deal with lack of course data by always being optimistic.
        if (maybe_field_to_course.isEmpty()) {
            maybe_field_to_course = maybe_field_to_goal;
        }
        if (maybe_field_to_goal.isEmpty()) {
            maybe_field_to_goal = maybe_field_to_course;
        }
        if (maybe_field_to_goal.isEmpty() && maybe_field_to_course.isEmpty()) {
            // Course doesn't matter.
            maybe_field_to_course = maybe_field_to_goal = Optional.of(GeometryUtil.kRotationIdentity);
        }
        Rotation2d field_to_course = maybe_field_to_course.get();
        Rotation2d robot_to_course = fieldToRobot.getRotation().unaryMinus().rotateBy(field_to_course);

        // Convert goal course to be relative to current course.
        // course_to_goal = course_to_field * field_to_goal
        Rotation2d course_to_goal = field_to_course.unaryMinus().rotateBy(maybe_field_to_goal.get());

        // Rotate error to be aligned to current course.
        // Error is in robot (heading) frame. Need to rotate it to be in course frame.
        // course_to_error = robot_to_course.inverse() * robot_to_error
        Translation2d linear_error_course_relative = GeometryUtil
                .transformBy(GeometryUtil.fromRotation(robot_to_course), mError).getTranslation();

        // Compute time-varying gain parameter.
        final double k = 2.0 * kZeta * Math.sqrt(
                kBeta * goal_linear_velocity * goal_linear_velocity + goal_angular_velocity * goal_angular_velocity);

        // Compute error components.
        final double angle_error_rads = course_to_goal.getRadians();
        final double sin_x_over_x = MathUtil.epsilonEquals(angle_error_rads, 0.0, 1E-2) ? 1.0
                : course_to_goal.getSin() / angle_error_rads;
        double adjusted_linear_velocity = goal_linear_velocity * course_to_goal.getCos()
                + k * linear_error_course_relative.getX();
        double adjusted_angular_velocity = goal_angular_velocity + k * angle_error_rads
                + goal_linear_velocity * kBeta * sin_x_over_x * linear_error_course_relative.getY();

        final double kThetaKp = 5.0; // Units are rad/s per rad of error.
        double heading_rate = goal_linear_velocity * fieldToGoal.state().getHeadingRate()
                + kThetaKp * mError.getRotation().getRadians();

        // Create a course-relative Twist2d.
        Twist2d adjusted_course_relative_velocity = new Twist2d(adjusted_linear_velocity, 0.0,
                adjusted_angular_velocity - heading_rate);
        // See where that takes us in one dt.
        final double kNominalDt = DriveMotionPlanner.kLooperDt;
        Pose2d adjusted_course_to_goal = new Pose2d()
                .exp(GeometryUtil.scale(adjusted_course_relative_velocity, kNominalDt));

        // Now rotate to be robot-relative.
        // robot_to_goal = robot_to_course * course_to_goal
        Translation2d adjusted_robot_to_goal = GeometryUtil
                .transformBy(GeometryUtil.fromRotation(robot_to_course), adjusted_course_to_goal).getTranslation()
                .times(1.0 / kNominalDt);

        return new ChassisSpeeds(
                adjusted_robot_to_goal.getX(),
                adjusted_robot_to_goal.getY(),
                heading_rate);
    }

}
