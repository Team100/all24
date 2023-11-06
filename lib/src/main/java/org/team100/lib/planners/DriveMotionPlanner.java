package org.team100.lib.planners;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.controller.DriveFeedforwardController;
import org.team100.lib.controller.DrivePIDController;
import org.team100.lib.controller.DrivePursuitController;
import org.team100.lib.controller.DriveRamseteController;
import org.team100.lib.controller.Lookahead;
import org.team100.lib.controller.SynchronousPIDF;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.path.Path;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.swerve.SwerveKinematicLimits;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.SwerveDriveDynamicsConstraint;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingUtil;
import org.team100.lib.timing.YawRateConstraint;
import org.team100.lib.trajectory.Trajectory;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryUtil;
import org.team100.lib.util.MathUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** This is derived from 254's 2023 version */
public class DriveMotionPlanner {
    private static final double kMaxDx = 0.0127; // m
    private static final double kMaxDy = 0.0127; // m
    private static final double kMaxDTheta = Math.toRadians(1.0);

    public static final double kPathLookaheadTime = 0.25;
    public static final double kPathMinLookaheadDistance = 12.0;
    // i think that in 254 2023 these were just wrong, not updated since they stopped using inches.
    // so i kinda fixed them up a bit.
    public static final double kAdaptivePathMinLookaheadDistance = 0.1;
    public static final double kAdaptivePathMaxLookaheadDistance = 0.1;
    public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;
    public static final double kMaxVelocityMetersPerSecond = 4.959668;

    // TODO: what loop time to use?
    public static final double kLooperDt = 0.02;


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

    final SwerveDriveKinematics swerve_kinematics_;
    final SwerveKinematicLimits swerve_kinematic_limits_;

    TrajectoryTimeIterator mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedPose mLastSetpoint = null;
    public TimedPose mSetpoint = new TimedPose(new Pose2dWithMotion());
    Pose2d mError = GeometryUtil.kPose2dIdentity;

    Translation2d mTranslationalError = GeometryUtil.kTranslation2dIdentity;
    Rotation2d mPrevHeadingError = GeometryUtil.kRotationIdentity;
    Pose2d mCurrentState = GeometryUtil.kPose2dIdentity;

    double mCurrentTrajectoryLength = 0.0;
    double mTotalTime = Double.POSITIVE_INFINITY;
    double mStartTime = Double.POSITIVE_INFINITY;
    ChassisSpeeds mOutput = new ChassisSpeeds();

    Lookahead mSpeedLookahead = null;

    // PID controllers for path following
    SynchronousPIDF mXPIDF;
    SynchronousPIDF mYPIDF;
    SynchronousPIDF mHeadingPIDF;

    double mDt = 0.0;

    private final DriveRamseteController m_ramsete = new DriveRamseteController();
    private final DrivePIDController m_pid = new DrivePIDController();
    private final DrivePursuitController m_pursuit = new DrivePursuitController();
    private final DriveFeedforwardController m_ff = new DriveFeedforwardController();

    public DriveMotionPlanner(SwerveDriveKinematics kinematics, SwerveKinematicLimits kinematic_limits) {
        swerve_kinematics_ = kinematics;
        swerve_kinematic_limits_ = kinematic_limits;
    }

    public void setTrajectory(final TrajectoryTimeIterator trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        mLastSetpoint = null;
        mSpeedLookahead = new Lookahead(kAdaptivePathMinLookaheadDistance, kAdaptivePathMaxLookaheadDistance, 0.0, kMaxVelocityMetersPerSecond);
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
        mTranslationalError = GeometryUtil.kTranslation2dIdentity;
        mPrevHeadingError = GeometryUtil.kRotationIdentity;
        mLastSetpoint = null;
        mOutput = new ChassisSpeeds();
        mLastTime = Double.POSITIVE_INFINITY;
        m_pursuit.reset();
        m_ff.reset();
        m_pid.reset();
        m_ramsete.reset();
    }

    public Trajectory generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint> constraints,
            double max_vel,  // m/s
            double max_accel,  // m/s^2
            double max_voltage) {
        return generateTrajectory(reversed, waypoints, headings, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
    }

    public Trajectory generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint> constraints,
            double start_vel,
            double end_vel,
            double max_vel,  // m/s
            double max_accel,  // m/s^2
            double max_voltage) {
        List<Pose2d> waypoints_maybe_flipped = waypoints;
        List<Rotation2d> headings_maybe_flipped = headings;
        final Pose2d flip = GeometryUtil.fromRotation(new Rotation2d(-1, 0));
        if (reversed) {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            headings_maybe_flipped = new ArrayList<>(headings.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypoints_maybe_flipped.add(GeometryUtil.transformBy(waypoints.get(i), flip));
                headings_maybe_flipped.add(headings.get(i).rotateBy(flip.getRotation()));
            }
        }

        // Create a trajectory from splines.
        Path trajectory = TrajectoryUtil.trajectoryFromWaypointsAndHeadings(
                waypoints_maybe_flipped, headings_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

        if (reversed) {
            List<Pose2dWithMotion> flipped_points = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                flipped_points.add(new Pose2dWithMotion(GeometryUtil.transformBy(trajectory.getPoint(i).state().getPose(), flip), -trajectory
                        .getPoint(i).state().getCurvature(), trajectory.getPoint(i).state().getDCurvatureDs()));
            }
            trajectory = new Path(flipped_points);
        }

        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        final SwerveDriveDynamicsConstraint drive_constraints = new
                SwerveDriveDynamicsConstraint(swerve_kinematics_, swerve_kinematic_limits_);
        final double kMaxYawRateRadS = 3.0;
        final YawRateConstraint yaw_constraint = new YawRateConstraint(kMaxYawRateRadS);
        final double kMaxCentripetalAccel = 10.0;//1.524;  // m/s^2
        final CentripetalAccelerationConstraint centripetal_accel_constraint = new CentripetalAccelerationConstraint(kMaxCentripetalAccel);

        List<TimingConstraint> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        all_constraints.add(yaw_constraint);
        all_constraints.add(centripetal_accel_constraint);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }

        // Generate the timed trajectory.
        PathDistanceSampler distance_view = new PathDistanceSampler(trajectory);
        return TimingUtil.timeParameterizeTrajectory(reversed, 
                        distance_view, kMaxDx, all_constraints, start_vel, end_vel, max_vel, max_accel);
    }

    public ChassisSpeeds update(double timestamp, Pose2d current_state, Twist2d current_velocity) {
        if (mCurrentTrajectory == null) return null;
        t.log("/planner/current state", current_state);

        if (!Double.isFinite(mLastTime)) mLastTime = timestamp;
        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint sample_point;
        mCurrentState = current_state;

        if (!isDone()) {
            // Compute error in robot frame
            mPrevHeadingError = mError.getRotation();
            // 254 calculates error using the *previous* setpoint.  i think this is a big mistake,
            // which isn't obvious unless the previous and current are far apart, i.e. if dt is very large, e.g. in a unit test.
            // mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state), mSetpoint.state().getPose());

            if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
                sample_point = mCurrentTrajectory.advance(mDt);
                // RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
                mSetpoint = sample_point.state();
                mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state), mSetpoint.state().getPose());

                mOutput = m_ff.updateFeedforward(current_state, mSetpoint);
            } else if (mFollowerType == FollowerType.RAMSETE) {
                sample_point = mCurrentTrajectory.advance(mDt);
                // RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
                mSetpoint = sample_point.state();
                mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state), mSetpoint.state().getPose());

                mOutput = m_ramsete.updateRamsete(sample_point.state(), current_state, current_velocity, mError);
            } else if (mFollowerType == FollowerType.PID) {
                sample_point = mCurrentTrajectory.advance(mDt);
                t.log("/planner/sample point", sample_point);
                
                // RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
                
                mSetpoint = sample_point.state();
                mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state), mSetpoint.state().getPose());

                t.log("/planner/setpoint", mSetpoint);

                final double velocity_m = mSetpoint.velocityM_S();
                t.log("/planner/setpoint velocity", velocity_m);

                // Field relative
                var course = mSetpoint.state().getCourse();
                Rotation2d motion_direction = course.isPresent() ? course.get() : GeometryUtil.kRotationIdentity;
                // Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.
                motion_direction = current_state.getRotation().unaryMinus().rotateBy(motion_direction);
                t.log("/planner/motion direction", motion_direction);

                ChassisSpeeds chassis_speeds = new ChassisSpeeds(
                        motion_direction.getCos() * velocity_m,
                        motion_direction.getSin() * velocity_m,
                        // Need unit conversion because Pose2dWithMotion heading rate is per unit distance.
                        velocity_m * mSetpoint.state().getHeadingRate());
                t.log("/planner/chassis speeds", chassis_speeds);

                // PID is in robot frame
                mOutput = m_pid.updatePIDChassis(chassis_speeds, mError);
            } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
                double searchStepSize = 1.0;
                double previewQuantity = 0.0;
                double searchDirection = 1.0;
                double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
                double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
                searchDirection = Math.signum(reverseDistance - forwardDistance);
                while(searchStepSize > 0.001){
                    if(MathUtil.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.01)) break;
                    while(/* next point is closer than current point */ distance(current_state, previewQuantity + searchStepSize*searchDirection) <
                            distance(current_state, previewQuantity)) {
                        /* move to next point */
                        previewQuantity += searchStepSize*searchDirection;
                    }
                    searchStepSize /= 10.0;
                    searchDirection *= -1;
                }
                sample_point = mCurrentTrajectory.advance(previewQuantity);
                // RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
                mSetpoint = sample_point.state();
                mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state), mSetpoint.state().getPose());

                mOutput = m_pursuit.updatePurePursuit(current_state,0.0,
                 mError,
                 mCurrentTrajectory,
                 mSetpoint,
                 mSpeedLookahead,
                 mIsReversed,
                 mCurrentTrajectoryLength,
                 mPrevHeadingError,
                 mDt
                );
            }
        } else {
            mOutput = new ChassisSpeeds();
        }

        return mOutput;
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    public synchronized Translation2d getTranslationalError() {
        return new Translation2d(
                mError.getTranslation().getX(),
                mError.getTranslation().getY());
    }

    public synchronized Rotation2d getHeadingError() {
        return mError.getRotation();
    }

    private double distance(Pose2d current_state, double additional_progress){
        return GeometryUtil.distance(mCurrentTrajectory.preview(additional_progress).state().state().getPose(), current_state);
    }

    public synchronized TimedPose getSetpoint() {
        return mSetpoint;
    }
}
