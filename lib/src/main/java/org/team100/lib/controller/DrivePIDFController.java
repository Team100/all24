package org.team100.lib.controller;

import java.util.Optional;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.BooleanSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.Pose2dLogger;
import org.team100.lib.logging.SupplierLogger2.TimedPoseLogger;
import org.team100.lib.logging.SupplierLogger2.TrajectorySamplePointLogger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Follow a 254 trajectory using velocity feedforward with optional positional
 * feedback.
 */
public class DrivePIDFController implements DriveMotionController {
    private static final double kPCartV = 1.0;
    private static final double kPThetaV = 1.0;

    /** Log exists so multiple controllers can use the same keys. */
    public static class Log {
        private final Pose2dLogger m_log_measurement;
        private final TimedPoseLogger m_log_setpoint;
        private final BooleanSupplierLogger2 m_log_is_mt;
        private final TrajectorySamplePointLogger m_log_sample;

        public Log(SupplierLogger2 parent) {
            SupplierLogger2 log = parent.child("DrivePIDFController");
            m_log_measurement = log.pose2dLogger(Level.DEBUG, "measurement");
            m_log_setpoint = log.timedPoseLogger(Level.DEBUG, "setpoint");
            m_log_is_mt = log.booleanLogger(Level.TRACE, "IS MT");
            m_log_sample = log.trajectorySamplePointLogger(Level.DEBUG, "sample point");
        }
    }

    private final Log m_log;
    private final boolean m_feedforwardOnly;
    private final double m_kPCart;
    private final double m_kPTheta;
    private final DriveMotionControllerUtil m_util;

    private TrajectoryTimeIterator m_iter;
    private double m_prevTimeS;

    /** Use the factory. */
    DrivePIDFController(
            Log log,
            DriveMotionControllerUtil util,
            boolean feedforwardOnly,
            double kPCart,
            double kPTheta) {
        m_log = log;
        m_feedforwardOnly = feedforwardOnly;
        m_kPCart = kPCart;
        m_kPTheta = kPTheta;
        m_util = util;
    }

    @Override
    public void setTrajectory(final TrajectoryTimeIterator iter) {
        m_iter = iter;
        m_prevTimeS = Double.POSITIVE_INFINITY;
    }

    /** Makes no attempt to produce feasible output. */
    @Override
    public ChassisSpeeds update(
            double timeS,
            Pose2d measurement,
            ChassisSpeeds currentRobotRelativeVelocity) {
        if (m_iter == null)
            return new ChassisSpeeds();

        m_log.m_log_measurement.log(() -> measurement);

        Optional<TimedPose> optionalSetpoint = getSetpoint(timeS);
        if (!optionalSetpoint.isPresent()) {
            return new ChassisSpeeds();
        }
        TimedPose setpoint = optionalSetpoint.get();
        SmartDashboard.putNumber("setpointX", setpoint.state().getPose().getX());
        m_log.m_log_setpoint.log(() -> setpoint);

        ChassisSpeeds u_FF = m_util.feedforward(measurement, setpoint);
        if (m_feedforwardOnly)
            return u_FF;

        ChassisSpeeds u_FB;
        if (Experiments.instance.enabled(Experiment.FullStateTrajectoryFollower)) {
            u_FB = m_util.fullFeedback(
                    measurement,
                    setpoint,
                    m_kPCart,
                    m_kPTheta,
                    currentRobotRelativeVelocity,
                    kPCartV,
                    kPThetaV);
        } else {
            u_FB = m_util.feedback(
                    measurement,
                    setpoint,
                    m_kPCart,
                    m_kPTheta);
        }

        return u_FF.plus(u_FB);
    }

    double dt(double timestamp) {
        if (!Double.isFinite(m_prevTimeS))
            m_prevTimeS = timestamp;
        double mDt = timestamp - m_prevTimeS;
        m_prevTimeS = timestamp;
        return mDt;
    }

    Optional<TimedPose> getSetpoint(double timestamp) {
        double mDt = dt(timestamp);

        Optional<TrajectorySamplePoint> sample_point = m_iter.advance(mDt);
        if (!sample_point.isPresent()) {
            m_log.m_log_is_mt.log(() -> true);
            return Optional.empty();
        }
        m_log.m_log_sample.log(sample_point::get);
        return Optional.of(sample_point.get().state());
    }

    @Override
    public boolean isDone() {
        // this used to also wait for the pose to match the goal, but that
        // took more time, so it's gone.
        return m_iter != null && m_iter.isDone();
    }
}
