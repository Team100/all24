package org.team100.lib.controller;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveFeedforwardController {
    public static final Telemetry t = Telemetry.get();

    private Pose2d mError = GeometryUtil.kPose2dIdentity;

    public Pose2d getError() {
        return mError;
    }

    private TrajectoryTimeIterator mCurrentTrajectory;
    public TimedPose mSetpoint = new TimedPose(new Pose2dWithMotion());
    double mLastTime = Double.POSITIVE_INFINITY;

    public void setTrajectory(final TrajectoryTimeIterator trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();

    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    public void reset() {
        mError = GeometryUtil.kPose2dIdentity;
        mLastTime = Double.POSITIVE_INFINITY;

    }

    public ChassisSpeeds updateFeedforward(final double timestamp, final Pose2d current_state) {
        if (mCurrentTrajectory == null)
            return null;
        t.log("/planner/current state", current_state);
        if (isDone()) {
            return new ChassisSpeeds();
        }

        if (!Double.isFinite(mLastTime))
            mLastTime = timestamp;
        final double mDt = timestamp - mLastTime;
        mLastTime = timestamp;

        TrajectorySamplePoint sample_point = mCurrentTrajectory.advance(mDt);
        t.log("/ff_planner/sample point", sample_point);
        mSetpoint = sample_point.state();
        t.log("/ff_planner/setpoint", mSetpoint);

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

    public synchronized Translation2d getTranslationalError() {
        return new Translation2d(
                getError().getTranslation().getX(),
                getError().getTranslation().getY());
    }

    public synchronized Rotation2d getHeadingError() {
        return getError().getRotation();
    }


}
