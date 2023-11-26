package org.team100.lib.controller;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This originated in DriveMotionPlanner, which included several
 * controllers.
 */
public class DrivePursuitController implements DriveMotionController {
    public static final Telemetry t = Telemetry.get();

    private static final double defaultCook = 0.5;
    private static final double kPathLookaheadTime = 0.25;
    private static final double kPathMinLookaheadDistance = 12.0;
    private static final double kAdaptivePathMinLookaheadDistance = 0.1;
    private static final double kAdaptivePathMaxLookaheadDistance = 0.1;
    private static final double kLookaheadSearchDt = 0.01;
    private static final double kMaxVelocityMetersPerSecond = 4.959668;

    private boolean useDefaultCook = true;
    private Lookahead mSpeedLookahead = null;
    // used for the "D" term of the heading controller
    private Rotation2d mPrevHeadingError = GeometryUtil.kRotationZero;
    private Pose2d mError = GeometryUtil.kPoseZero;
    private TrajectoryTimeIterator mCurrentTrajectory;
    private double mCurrentTrajectoryLength = 0.0;
    private TimedPose mSetpoint = new TimedPose(new Pose2dWithMotion());
    private double mLastTime = Double.POSITIVE_INFINITY;
    private boolean mIsReversed = false;

    @Override
    public void setTrajectory(final TrajectoryTimeIterator trajectory) {
        mCurrentTrajectory = trajectory;
        mCurrentTrajectoryLength = mCurrentTrajectory.trajectory().getLastPoint().state().getTimeS();
        mSetpoint = trajectory.getState();

        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getPoint(i).state().velocityM_S() > Math100.EPSILON) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getPoint(i).state().velocityM_S() < -Math100.EPSILON) {
                mIsReversed = true;
                break;
            }
        }
        useDefaultCook = true;
        mSpeedLookahead = new Lookahead(kAdaptivePathMinLookaheadDistance, kAdaptivePathMaxLookaheadDistance, 0.0,
                kMaxVelocityMetersPerSecond);
        mPrevHeadingError = GeometryUtil.kRotationZero;
        mError = GeometryUtil.kPoseZero;
        mLastTime = Double.POSITIVE_INFINITY;
    }

    @Override
    public ChassisSpeeds update(double timestamp, Pose2d current_state, Twist2d current_velocity) {
        return updatePurePursuit(timestamp, current_state, 0.0);
    }

    public ChassisSpeeds updatePurePursuit(final double timestamp,
            final Pose2d current_state,
            final double feedforwardOmegaRadiansPerSecond) {
        if (mCurrentTrajectory == null)
            return null;

        t.log(Level.DEBUG, "/planner/current state", current_state);
        if (isDone()) {
            return new ChassisSpeeds();
        }

        if (!Double.isFinite(mLastTime))
            mLastTime = timestamp;
        final double mDt = timestamp - mLastTime;
        mLastTime = timestamp;

        double previewQuantity = DrivePursuitController.previewDt(mCurrentTrajectory, current_state);

        TrajectorySamplePoint sample_point = mCurrentTrajectory.advance(previewQuantity);
        t.log(Level.DEBUG, "/pursuit_planner/sample point", sample_point);
        mSetpoint = sample_point.state();
        t.log(Level.DEBUG, "/pursuit_planner/setpoint", mSetpoint);

        mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state), mSetpoint.state().getPose());

        t.log(Level.DEBUG, "/pursuit_planner/error", mError);

        double lookahead_time = kPathLookaheadTime;

        TimedPose lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
        t.log(Level.DEBUG, "/pursuit_planner/lookahead state", lookahead_state);

        double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.velocityM_S());
        // Find the Point on the Trajectory that is Lookahead Distance Away
        while (actual_lookahead_distance < adaptive_lookahead_distance &&
                mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        }

        // If the Lookahead Point's Distance is less than the Lookahead Distance
        // transform it so it is the lookahead distance away
        if (actual_lookahead_distance < adaptive_lookahead_distance) {
            lookahead_state = new TimedPose(
                    new Pose2dWithMotion(
                            GeometryUtil.transformBy(lookahead_state.state()
                                    .getPose(),
                                    GeometryUtil.fromTranslation(new Translation2d(
                                            (mIsReversed ? -1.0 : 1.0) * (kPathMinLookaheadDistance -
                                                    actual_lookahead_distance),
                                            0.0))),
                            0.0),
                    lookahead_state.getTimeS(), lookahead_state.velocityM_S(), lookahead_state.acceleration());
        }
        t.log(Level.DEBUG, "/pursuit_planner/updated lookahead state", lookahead_state);

        // Find the vector between robot's current position and the lookahead state
        Translation2d lookaheadTranslation = lookahead_state.state().getTranslation()
                .minus(current_state.getTranslation());
        t.log(Level.DEBUG, "/pursuit_planner/lookahead translation", lookaheadTranslation);

        // Set the steering direction as the direction of the vector
        Rotation2d steeringDirection = lookaheadTranslation.getAngle();

        // Convert from field-relative steering direction to robot-relative
        steeringDirection = steeringDirection.rotateBy(GeometryUtil.inverse(current_state).getRotation());

        // Use the Velocity Feedforward of the Closest Point on the Trajectory
        double normalizedSpeed = Math.abs(mSetpoint.velocityM_S()) / kMaxVelocityMetersPerSecond;

        // The Default Cook is the minimum speed to use. So if a feedforward speed is
        // less than defaultCook, the robot will drive at the defaultCook speed
        if (normalizedSpeed > defaultCook || mSetpoint.getTimeS() > (mCurrentTrajectoryLength / 2.0)) {
            useDefaultCook = false;
        }
        if (useDefaultCook) {
            normalizedSpeed = defaultCook;
        }

        // Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate
        // (Vx, Vy) in Robot Frame
        final Translation2d steeringVector = new Translation2d(steeringDirection.getCos() * normalizedSpeed,
                steeringDirection.getSin() * normalizedSpeed);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                steeringVector.getX() * kMaxVelocityMetersPerSecond,
                steeringVector.getY() * kMaxVelocityMetersPerSecond,
                feedforwardOmegaRadiansPerSecond);

        t.log(Level.DEBUG, "/pursuit_planner/pursuit speeds", chassisSpeeds);

        // Use the PD-Controller for To Follow the Time-Parametrized Heading
        final double kThetakP = 3.5;
        final double kThetakD = 0.0;
        final double kPositionkP = 2.0;

        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond
                + kPositionkP * mError.getTranslation().getX();
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond
                + kPositionkP * mError.getTranslation().getY();
        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
                + (kThetakP * mError.getRotation().getRadians())
                + kThetakD * ((mError.getRotation().getRadians() - mPrevHeadingError.getRadians()) / mDt);

        // save rotation error for next iteration
        mPrevHeadingError = mError.getRotation();

        return chassisSpeeds;
    }

    @Override
    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    // for testing
    TimedPose getSetpoint() {
        return mSetpoint;
    }

    // for testing
    Pose2d getError() {
        return mError;
    }

    private static double distance(TrajectoryTimeIterator mCurrentTrajectory, Pose2d current_state,
            double additional_progress) {
        return GeometryUtil.distance(mCurrentTrajectory.preview(additional_progress).state().state().getPose(),
                current_state);
    }

    private static double previewDt(TrajectoryTimeIterator mCurrentTrajectory, Pose2d current_state) {
        double searchStepSize = 1.0;
        double previewQuantity = 0.0;
        double searchDirection = 1.0;
        double forwardDistance = distance(mCurrentTrajectory, current_state, previewQuantity + searchStepSize);
        double reverseDistance = distance(mCurrentTrajectory, current_state, previewQuantity - searchStepSize);
        searchDirection = Math.signum(reverseDistance - forwardDistance);
        while (searchStepSize > 0.001) {
            if (Math100.epsilonEquals(distance(mCurrentTrajectory, current_state, previewQuantity), 0.0, 0.01))
                break;
            while (/* next point is closer than current point */ distance(mCurrentTrajectory, current_state,
                    previewQuantity + searchStepSize * searchDirection) < distance(mCurrentTrajectory, current_state,
                            previewQuantity)) {
                /* move to next point */
                previewQuantity += searchStepSize * searchDirection;
            }
            searchStepSize /= 10.0;
            searchDirection *= -1;
        }
        return previewQuantity;
    }
}
