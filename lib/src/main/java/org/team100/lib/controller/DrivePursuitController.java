package org.team100.lib.controller;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.ChassisSpeedsLogger;
import org.team100.lib.logging.SupplierLogger2.Pose2dLogger;
import org.team100.lib.logging.SupplierLogger2.TimedPoseLogger;
import org.team100.lib.logging.SupplierLogger2.TrajectorySamplePointLogger;
import org.team100.lib.logging.SupplierLogger2.Translation2dLogger;
import org.team100.lib.logging.SupplierLogger2.Twist2dLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Follow a 254 trajectory using pure pursuit.
 * 
 * The update timestamp is ignored; the controller finds the closest point on
 * the trajectory to the current pose and steers towards a near-future sample
 * from there. So this controller deals with disturbance differently than a
 * simple timed follower: for example, if blocked, it will keep trying to get to
 * the next reasonable near point on the trajectory, whereas a timed follower
 * would target far away points.
 * 
 * This originated in 254's DriveMotionPlanner, which included several
 * controllers.
 */
public class DrivePursuitController implements DriveMotionController {
    public static final double EPSILON = 1e-6;

    private static final double kPathLookaheadTime = 0.25;
    private static final double kPathMinLookaheadDistance = 12.0;
    private static final double kAdaptivePathMinLookaheadDistance = 0.1;
    private static final double kAdaptivePathMaxLookaheadDistance = 0.1;
    private static final double kLookaheadSearchDt = 0.01;
    // 254 calls this "default cook"
    private static final double kMinSpeed = 0.5;

    // feedback control constants
    private static final double kThetakP = 3.5;
    private static final double kPositionkP = 2.0;

    private final SwerveKinodynamics m_limits;
    public final SupplierLogger2 m_logger;
    // LOGGERS
    private final ChassisSpeedsLogger m_log_u_FF;
    private final ChassisSpeedsLogger m_log_u_FB;
    private final Pose2dLogger m_log_measurement;
    private final TimedPoseLogger m_log_setpoint;
    private final TimedPoseLogger m_log_lookahead;
    private final TimedPoseLogger m_log_updated_lookahead;
    private final Translation2dLogger m_log_lookahead_translation;
    private final Twist2dLogger m_log_error;
    private final TrajectorySamplePointLogger m_log_sample;

    private Lookahead mSpeedLookahead = null;

    private TrajectoryTimeIterator m_iter;
    private boolean mIsReversed = false;

    /** Min speed is used at the start of a trajectory only. */
    private boolean useMinSpeed;

    /** Use the factory. */
    DrivePursuitController(SupplierLogger2 parent, SwerveKinodynamics limits) {
        m_limits = limits;
        m_logger = parent.child(this);
        m_log_u_FF = parent.chassisSpeedsLogger(Level.TRACE, "u_FF");
        m_log_u_FB = parent.chassisSpeedsLogger(Level.TRACE, "u_FB");
        m_log_measurement = m_logger.pose2dLogger(Level.TRACE, "current state");
        m_log_setpoint = m_logger.timedPoseLogger(Level.TRACE, "setpoint");
        m_log_lookahead = m_logger.timedPoseLogger(Level.TRACE, "lookahead state");
        m_log_updated_lookahead = m_logger.timedPoseLogger(Level.TRACE, "updated lookahead state");
        m_log_lookahead_translation = m_logger.translation2dLogger(Level.TRACE, "lookahead translation");
        m_log_error = m_logger.twist2dLogger(Level.TRACE, "pursuit error");
        m_log_sample = m_logger.trajectorySamplePointLogger(Level.TRACE, "sample point");
    }

    @Override
    public void setTrajectory(TrajectoryTimeIterator trajectory) {
        m_iter = trajectory;
        useMinSpeed = true;

        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getPoint(i).state().velocityM_S() > EPSILON) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getPoint(i).state().velocityM_S() < -EPSILON) {
                mIsReversed = true;
                break;
            }
        }
        mSpeedLookahead = new Lookahead(kAdaptivePathMinLookaheadDistance, kAdaptivePathMaxLookaheadDistance, 0.0,
                m_limits.getMaxDriveVelocityM_S());
    }

    /**
     * 
     * @param timestamp        ignored
     * @param measurement      measured pose
     * @param current_velocity ignored
     * @return velocity control input
     */
    @Override
    public ChassisSpeeds update(double timestamp, Pose2d measurement, ChassisSpeeds current_velocity) {
        if (m_iter == null) {
            Util.warn("Null iter!");
            return null;
        }

        m_log_measurement.log(() -> measurement);
        if (isDone()) {
            return new ChassisSpeeds();
        }

        Optional<TimedPose> optionalSetpoint = getSetpoint(measurement);
        if (!optionalSetpoint.isPresent()) {
            Util.warn("No setpoint!");
            return new ChassisSpeeds();
        }
        TimedPose mSetpoint = optionalSetpoint.get();
        m_log_setpoint.log(() -> mSetpoint);

        double lookahead_time = kPathLookaheadTime;

        Optional<TrajectorySamplePoint> preview = m_iter.preview(lookahead_time);
        if (!preview.isPresent()) {
            Util.warn("No preview!");
            return new ChassisSpeeds();
        }

        TimedPose lookahead_state = preview.get().state();
        m_log_lookahead.log(() -> preview.get().state());

        double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.velocityM_S());

        // Find the Point on the Trajectory that is Lookahead Distance Away
        while (actual_lookahead_distance < adaptive_lookahead_distance &&
                m_iter.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            Optional<TrajectorySamplePoint> preview2 = m_iter.preview(lookahead_time);
            if (!preview2.isPresent()) {
                Util.warn("No Preview2!");
                return new ChassisSpeeds();
            }
            lookahead_state = preview2.get().state();
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
                                            0.0)))),
                    lookahead_state.getTimeS(), lookahead_state.velocityM_S(), lookahead_state.acceleration());
        }
        final TimedPose updatedLookaheadState = lookahead_state;
        m_log_updated_lookahead.log(() -> updatedLookaheadState);

        // Find the vector between robot's current position and the lookahead state
        Translation2d lookaheadTranslation = lookahead_state.state().getTranslation()
                .minus(measurement.getTranslation());
        m_log_lookahead_translation.log(() -> lookaheadTranslation);

        // Set the steering direction as the direction of the vector
        Rotation2d steeringDirection = lookaheadTranslation.getAngle();

        // Convert from field-relative steering direction to robot-relative
        steeringDirection = steeringDirection.rotateBy(GeometryUtil.inverse(measurement).getRotation());

        // Use the Velocity Feedforward of the Closest Point on the Trajectory
        double normalizedSpeed = Math.abs(mSetpoint.velocityM_S()) / m_limits.getMaxDriveVelocityM_S();

        if (normalizedSpeed > kMinSpeed) {
            // latch it off for this trajectory to avoid running off the end.
            // min speed just affects the start.
            useMinSpeed = false;
        }
        if (useMinSpeed) {
            normalizedSpeed = Math.max(normalizedSpeed, kMinSpeed);
        }

        // Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate
        // (Vx, Vy) in Robot Frame
        final Translation2d steeringVector = new Translation2d(
                steeringDirection.getCos() * normalizedSpeed,
                steeringDirection.getSin() * normalizedSpeed);

        ChassisSpeeds u_FF = new ChassisSpeeds(
                steeringVector.getX() * m_limits.getMaxDriveVelocityM_S(),
                steeringVector.getY() * m_limits.getMaxDriveVelocityM_S(),
                0.0);
        m_log_u_FF.log(() -> u_FF);

        Twist2d errorTwist = DriveMotionControllerUtil.getErrorTwist(measurement, mSetpoint);
        m_log_error.log(() -> errorTwist);
        ChassisSpeeds u_FB = new ChassisSpeeds(
                kPositionkP * errorTwist.dx,
                kPositionkP * errorTwist.dy,
                kThetakP * errorTwist.dtheta);
        m_log_u_FB.log(() -> u_FB);

        ChassisSpeeds chassisSpeeds = u_FF.plus(u_FB);

        DriveUtil.checkSpeeds(chassisSpeeds);

        return chassisSpeeds;
    }

    /**
     * Return the trajectory sample closest to the given pose. Returns empty if
     * something goes wrong.
     */
    Optional<TimedPose> getSetpoint(final Pose2d measuredPose) {
        // time to get to the trajectory point closest to the current pose
        OptionalDouble previewQuantity = previewDt(m_iter, measuredPose);
        if (!previewQuantity.isPresent()) {
            return Optional.empty();
        }

        Optional<TrajectorySamplePoint> sample_point = m_iter.advance(previewQuantity.getAsDouble());
        if (!sample_point.isPresent()) {
            return Optional.empty();
        }
        m_log_sample.log(sample_point::get);
        return Optional.of(sample_point.get().state());
    }

    @Override
    public boolean isDone() {
        return m_iter != null && m_iter.isDone();
    }

    /**
     * Length of a constant-twist path between the current measured pose and a
     * sample point.
     */
    private static double distance(Pose2d pose, TrajectorySamplePoint preview) {
        Pose2d previewPose = preview.state().state().getPose();
        return GeometryUtil.distance(previewPose, pose);
    }

    private static OptionalDouble initialDir(TrajectoryTimeIterator iter, Pose2d pose) {
        Optional<TrajectorySamplePoint> fwdPreview = iter.preview(1.0);
        if (!fwdPreview.isPresent()) {
            return OptionalDouble.empty();
        }
        double fwd = distance(pose, fwdPreview.get());
        Optional<TrajectorySamplePoint> revPreview = iter.preview(-1.0);
        if (!revPreview.isPresent()) {
            return OptionalDouble.empty();
        }
        double rev = distance(pose, revPreview.get());
        // search the closer end first
        return OptionalDouble.of(Math.signum(rev - fwd));
    }

    /**
     * Find the preview time to reach the point on the trajectory closest to the
     * given pose. This doesn't require that the given pose be on the trajectory at
     * all, it just picks the nearest point.
     * 
     * NOTE: if the probe is at the end, we return a time that is past the end
     * rather than the end arrival time.
     * 
     * @param iter
     * @param pose probe pose
     * @return preview time in seconds
     */
    static OptionalDouble previewDt(TrajectoryTimeIterator iter, Pose2d pose) {
        OptionalDouble initialDir = initialDir(iter, pose);
        if (!initialDir.isPresent())
            return OptionalDouble.empty();
        double dir = initialDir.getAsDouble();

        double step = 1.0;
        double dt = 0.0;
        while (step > 0.001) {
            Optional<TrajectorySamplePoint> dtPreview = iter.preview(dt);
            if (!dtPreview.isPresent()) {
                return OptionalDouble.empty();
            }
            if (Math100.epsilonEquals(distance(pose, dtPreview.get()), 0.0, 0.01))
                break; // found the pose exactly
            while (true) {
                double probe = dt + step * dir;

                Optional<TrajectorySamplePoint> probePreview = iter.preview(probe);
                if (!probePreview.isPresent()) {
                    return OptionalDouble.empty();
                }
                double probeDist = distance(pose, probePreview.get());

                Optional<TrajectorySamplePoint> preview = iter.preview(dt);
                if (!preview.isPresent()) {
                    return OptionalDouble.empty();
                }
                double dtDist = distance(pose, preview.get());

                if (probeDist < dtDist) {
                    // probe is closer than current point, use the probe
                    dt = probe;
                } else {
                    // probe is worse
                    break;
                }
            }
            // try the other way, slower
            step /= 10.0;
            dir *= -1;
        }
        return OptionalDouble.of(dt);
    }

    @Override
    public String getGlassName() {
        return "DrivePursuitController";
    }

}
