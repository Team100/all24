package org.team100.lib.planners;

import org.team100.lib.controller.DriveFeedforwardController;
import org.team100.lib.controller.DrivePIDController;
import org.team100.lib.controller.DrivePursuitController;
import org.team100.lib.controller.DriveRamseteController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.util.MathUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** This is derived from 254's 2023 version */
public class DriveMotionPlanner {

    public static final Telemetry t = Telemetry.get();

    public enum FollowerType {
        FEEDFORWARD_ONLY,
        PID,
        PURE_PURSUIT,
        RAMSETE
    }

    FollowerType mFollowerType = FollowerType.PID;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }

    TrajectoryTimeIterator mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedPose mLastSetpoint = null;
    public TimedPose mSetpoint = new TimedPose(new Pose2dWithMotion());


    double mCurrentTrajectoryLength = 0.0;

    private final DriveRamseteController m_ramsete = new DriveRamseteController();
    private final DrivePIDController m_pid = new DrivePIDController();
    private final DrivePursuitController m_pursuit = new DrivePursuitController();
    private final DriveFeedforwardController m_ff = new DriveFeedforwardController();

    public DriveMotionPlanner() {
        // TODO remove this
    }

    public void setTrajectory(final TrajectoryTimeIterator trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        mLastSetpoint = null;
        mCurrentTrajectoryLength = mCurrentTrajectory.trajectory().getLastPoint().state().getTimeS();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getPoint(i).state().velocityM_S() > MathUtil.EPSILON) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getPoint(i).state().velocityM_S() < -MathUtil.EPSILON) {
                mIsReversed = true;
                break;
            }
        }
    }

    public void reset() {
        mLastSetpoint = null;
        mLastTime = Double.POSITIVE_INFINITY;
        m_pursuit.reset();
        m_ff.reset();
        m_pid.reset();
        m_ramsete.reset();
    }

    public ChassisSpeeds update(double timestamp, Pose2d current_state, Twist2d current_velocity) {
        if (mCurrentTrajectory == null)
            return null;
        t.log("/planner/current state", current_state);

        if (!Double.isFinite(mLastTime))
            mLastTime = timestamp;
        final double mDt = timestamp - mLastTime;
        mLastTime = timestamp;

        if (isDone()) {
            return new ChassisSpeeds();
        }

        if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {

            TrajectorySamplePoint sample_point = mCurrentTrajectory.advance(mDt);
            t.log("/ff_planner/sample point", sample_point);
            mSetpoint = sample_point.state();
            t.log("/ff_planner/setpoint", mSetpoint);

            return m_ff.updateFeedforward(current_state, mSetpoint);

        }
        if (mFollowerType == FollowerType.RAMSETE) {

            TrajectorySamplePoint sample_point = mCurrentTrajectory.advance(mDt);
            t.log("/ramsete_planner/sample point", sample_point);
            mSetpoint = sample_point.state();
            t.log("/ramsete_planner/setpoint", mSetpoint);

            return m_ramsete.updateRamsete(current_state, mSetpoint, sample_point.state(), current_state, current_velocity);

        }
        if (mFollowerType == FollowerType.PID) {

            TrajectorySamplePoint sample_point = mCurrentTrajectory.advance(mDt);
            t.log("/pid_planner/sample point", sample_point);
            mSetpoint = sample_point.state();
            t.log("/pid_planner/setpoint", mSetpoint);

            final double velocity_m = mSetpoint.velocityM_S();
            t.log("/pid_planner/setpoint velocity", velocity_m);

            // Field relative
            var course = mSetpoint.state().getCourse();
            Rotation2d motion_direction = course.isPresent() ? course.get() : GeometryUtil.kRotationIdentity;
            // Adjust course by ACTUAL heading rather than planned to decouple heading and
            // translation errors.
            motion_direction = current_state.getRotation().unaryMinus().rotateBy(motion_direction);
            t.log("/planner/motion direction", motion_direction);

            ChassisSpeeds chassis_speeds = new ChassisSpeeds(
                    motion_direction.getCos() * velocity_m,
                    motion_direction.getSin() * velocity_m,
                    // Need unit conversion because Pose2dWithMotion heading rate is per unit
                    // distance.
                    velocity_m * mSetpoint.state().getHeadingRate());
            t.log("/planner/chassis speeds", chassis_speeds);

            // PID is in robot frame
            return m_pid.updatePIDChassis(current_state, mSetpoint, chassis_speeds);

        }
        if (mFollowerType == FollowerType.PURE_PURSUIT) {

            double previewQuantity = DrivePursuitController.previewDt(mCurrentTrajectory, current_state);

            TrajectorySamplePoint sample_point = mCurrentTrajectory.advance(previewQuantity);
            t.log("/pid_planner/sample point", sample_point);
            mSetpoint = sample_point.state();
            t.log("/pid_planner/setpoint", mSetpoint);

            return m_pursuit.updatePurePursuit(current_state, 0.0,
                    mCurrentTrajectory,
                    mSetpoint,
                    mIsReversed,
                    mCurrentTrajectoryLength,
                    mDt);
        }

        return new ChassisSpeeds();
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    private Pose2d getError() {
        if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
            return m_ff.getError();
        }
        if (mFollowerType == FollowerType.RAMSETE) {
            return m_ramsete.getError();
        }
        if (mFollowerType == FollowerType.PID) {
            return m_pid.getError();
        }
        if (mFollowerType == FollowerType.PURE_PURSUIT) {
            return m_pursuit.getError();
        }
        return new Pose2d();
    }

    public synchronized Translation2d getTranslationalError() {
        return new Translation2d(
                getError().getTranslation().getX(),
                getError().getTranslation().getY());
    }

    public synchronized Rotation2d getHeadingError() {
        return getError().getRotation();
    }

    public synchronized TimedPose getSetpoint() {
        return mSetpoint;
    }
}
