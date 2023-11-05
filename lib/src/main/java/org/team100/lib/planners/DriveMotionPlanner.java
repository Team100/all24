package org.team100.lib.planners;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

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
    private static final double kLooperDt = 0.02;


    private final Telemetry t = Telemetry.get();

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

    private double defaultCook = 0.5;
    private boolean useDefaultCook = true;

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

    public DriveMotionPlanner(SwerveDriveKinematics kinematics, SwerveKinematicLimits kinematic_limits) {
        swerve_kinematics_ = kinematics;
        swerve_kinematic_limits_ = kinematic_limits;
    }

    public void setTrajectory(final TrajectoryTimeIterator trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        mLastSetpoint = null;
        useDefaultCook = true;
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

    protected ChassisSpeeds updateRamsete(TimedPose fieldToGoal, Pose2d fieldToRobot, Twist2d currentVelocity) {
        // Implements eqn. 5.12 from https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
        final double kBeta = 2.0;  // >0.
        final double kZeta = 0.7;  // Damping coefficient, [0, 1].

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
        Translation2d linear_error_course_relative = GeometryUtil.transformBy(GeometryUtil.fromRotation(robot_to_course), mError).getTranslation();

        // Compute time-varying gain parameter.
        final double k = 2.0 * kZeta * Math.sqrt(kBeta * goal_linear_velocity * goal_linear_velocity + goal_angular_velocity * goal_angular_velocity);

        // Compute error components.
        final double angle_error_rads = course_to_goal.getRadians();
        final double sin_x_over_x = MathUtil.epsilonEquals(angle_error_rads, 0.0, 1E-2) ?
                1.0 : course_to_goal.getSin() / angle_error_rads;
        double adjusted_linear_velocity = goal_linear_velocity * course_to_goal.getCos() + k * linear_error_course_relative.getX();
        double adjusted_angular_velocity = goal_angular_velocity + k * angle_error_rads + goal_linear_velocity * kBeta * sin_x_over_x * linear_error_course_relative.getY();

        final double kThetaKp = 5.0;  // Units are rad/s per rad of error.
        double heading_rate = goal_linear_velocity * fieldToGoal.state().getHeadingRate() + kThetaKp * mError.getRotation().getRadians();

        // Create a course-relative Twist2d.
        Twist2d adjusted_course_relative_velocity = new Twist2d(adjusted_linear_velocity, 0.0, adjusted_angular_velocity - heading_rate);
        // See where that takes us in one dt.
        final double kNominalDt = kLooperDt;
        Pose2d adjusted_course_to_goal = new Pose2d().exp(GeometryUtil.scale(adjusted_course_relative_velocity, kNominalDt));

        // Now rotate to be robot-relative.
        // robot_to_goal = robot_to_course * course_to_goal
        Translation2d adjusted_robot_to_goal = GeometryUtil.transformBy(GeometryUtil.fromRotation(robot_to_course), adjusted_course_to_goal).getTranslation().times(1.0 / kNominalDt);

        return new ChassisSpeeds(
            adjusted_robot_to_goal.getX(),
            adjusted_robot_to_goal.getY(),
            heading_rate);
    }

    protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
        // Feedback on longitudinal error (distance).
        final double kPathk = 2.4;
        //2.4;
        // Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
        //0.15;
        final double kPathKTheta = 2.4;

        t.log("/planner/error", mError);

        Twist2d pid_error = new Pose2d().log(mError);
        t.log("/planner/pid error", pid_error);

        chassisSpeeds.vxMetersPerSecond =
                chassisSpeeds.vxMetersPerSecond + kPathk * pid_error.dx;
        chassisSpeeds.vyMetersPerSecond =
                chassisSpeeds.vyMetersPerSecond + kPathk * pid_error.dy;
        chassisSpeeds.omegaRadiansPerSecond =
                chassisSpeeds.omegaRadiansPerSecond + kPathKTheta * pid_error.dtheta;
        return chassisSpeeds;
    }

    protected ChassisSpeeds updatePurePursuit(Pose2d current_state, double feedforwardOmegaRadiansPerSecond) {
        t.log("/planner/error", mError);

        double lookahead_time = kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;

        TimedPose lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
        t.log("/planner/lookahead state", lookahead_state);

        double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.velocityM_S());
        //Find the Point on the Trajectory that is Lookahead Distance Away
        while (actual_lookahead_distance < adaptive_lookahead_distance &&
                mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        }

        //If the Lookahead Point's Distance is less than the Lookahead Distance transform it so it is the lookahead distance away
        if (actual_lookahead_distance < adaptive_lookahead_distance) {
            lookahead_state = new TimedPose(
                new Pose2dWithMotion(
                   GeometryUtil.transformBy(lookahead_state.state()
                    .getPose(), GeometryUtil.fromTranslation(new Translation2d(
                            (mIsReversed ? -1.0 : 1.0) * (kPathMinLookaheadDistance -
                                    actual_lookahead_distance), 0.0))), 0.0), lookahead_state.getTimeS()
                    , lookahead_state.velocityM_S(), lookahead_state.acceleration());
        }
        t.log("/planner/updated lookahead state", lookahead_state);

        //Find the vector between robot's current position and the lookahead state
        Translation2d lookaheadTranslation = lookahead_state.state().getTranslation().minus(current_state.getTranslation());
        t.log("/planner/lookahead translation", lookaheadTranslation);

        //Set the steering direction as the direction of the vector
        Rotation2d steeringDirection = lookaheadTranslation.getAngle();

        //Convert from field-relative steering direction to robot-relative
        steeringDirection = steeringDirection.rotateBy(GeometryUtil.inverse(current_state).getRotation());

        //Use the Velocity Feedforward of the Closest Point on the Trajectory
        double normalizedSpeed = Math.abs(mSetpoint.velocityM_S()) / kMaxVelocityMetersPerSecond;

        //The Default Cook is the minimum speed to use. So if a feedforward speed is less than defaultCook, the robot will drive at the defaultCook speed
        if(normalizedSpeed > defaultCook || mSetpoint.getTimeS() > (mCurrentTrajectoryLength / 2.0)){
            useDefaultCook = false;
        }
        if(useDefaultCook){
            normalizedSpeed = defaultCook;
        }

        //Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate (Vx, Vy) in Robot Frame
        final Translation2d steeringVector = new Translation2d(steeringDirection.getCos() * normalizedSpeed, steeringDirection.getSin() * normalizedSpeed);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(steeringVector.getX() * kMaxVelocityMetersPerSecond, steeringVector.getY() * kMaxVelocityMetersPerSecond, feedforwardOmegaRadiansPerSecond);

        t.log("/planner/pursuit speeds", chassisSpeeds);

        //Use the PD-Controller for To Follow the Time-Parametrized Heading
        final double kThetakP = 3.5;
        final double kThetakD = 0.0;
        final double kPositionkP = 2.0;

        chassisSpeeds.vxMetersPerSecond =
                chassisSpeeds.vxMetersPerSecond + kPositionkP * mError.getTranslation().getX();
        chassisSpeeds.vyMetersPerSecond =
                chassisSpeeds.vyMetersPerSecond + kPositionkP * mError.getTranslation().getY();
        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + (kThetakP * mError.getRotation().getRadians()) + kThetakD * ((mError.getRotation().getRadians() - mPrevHeadingError.getRadians()) / mDt);
        return chassisSpeeds;
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

                final double velocity_m = mSetpoint.velocityM_S();
                // Field relative
                var course = mSetpoint.state().getCourse();
                Rotation2d motion_direction = course.isPresent() ? course.get() : GeometryUtil.kRotationIdentity;
                // Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.
                motion_direction = current_state.getRotation().unaryMinus().rotateBy(motion_direction);

                mOutput = new ChassisSpeeds(
                        motion_direction.getCos() * velocity_m,
                        motion_direction.getSin() * velocity_m,
                        // Need unit conversion because Pose2dWithMotion heading rate is per unit distance.
                        velocity_m * mSetpoint.state().getHeadingRate());
            } else if (mFollowerType == FollowerType.RAMSETE) {
                sample_point = mCurrentTrajectory.advance(mDt);
                // RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
                mSetpoint = sample_point.state();
                mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state), mSetpoint.state().getPose());

                mOutput = updateRamsete(sample_point.state(), current_state, current_velocity);
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
                mOutput = updatePIDChassis(chassis_speeds);
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

                mOutput = updatePurePursuit(current_state,0.0);
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
