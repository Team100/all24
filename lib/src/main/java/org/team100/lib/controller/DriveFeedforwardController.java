package org.team100.lib.controller;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveFeedforwardController {

    private Pose2d  mError = GeometryUtil.kPose2dIdentity;

    public Pose2d getError() {
        return mError;
    }


    public void reset() {
        mError = GeometryUtil.kPose2dIdentity;
    }

    public ChassisSpeeds updateFeedforward(
            final Pose2d current_state,
            final TimedPose mSetpoint) {

        mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state), mSetpoint.state().getPose());

        final double velocity_m = mSetpoint.velocityM_S();
        // Field relative
        var course = mSetpoint.state().getCourse();
        Rotation2d motion_direction = course.isPresent() ? course.get() : GeometryUtil.kRotationIdentity;
        // Adjust course by ACTUAL heading rather than planned to decouple heading and
        // translation errors.
        motion_direction = current_state.getRotation().unaryMinus().rotateBy(motion_direction);
        return new ChassisSpeeds(
                motion_direction.getCos() * velocity_m,
                motion_direction.getSin() * velocity_m,
                // Need unit conversion because Pose2dWithMotion heading rate is per unit
                // distance.
                velocity_m * mSetpoint.state().getHeadingRate());

    }

}
