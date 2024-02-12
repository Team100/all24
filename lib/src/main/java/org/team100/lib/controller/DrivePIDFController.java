package org.team100.lib.controller;

import java.util.Optional;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.util.Names;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Follow a 254 trajectory using velocity feedforward with optional positional
 * feedback.
 */
public class DrivePIDFController implements DriveMotionController {
    private static final double kTolerance = 0.05;
    public static final Telemetry t = Telemetry.get();
    private final boolean m_feedforwardOnly;
    private final String m_name;

    private TrajectoryTimeIterator m_iter;
    private double m_prevTimeS;
    private Pose2d error = new Pose2d();
    private double m_kPCart;
    private double m_kPTheta;


    public DrivePIDFController(boolean feedforwardOnly, double kPCart, double kPTheta) {
        m_feedforwardOnly = feedforwardOnly;
        m_kPCart = kPCart;
        m_kPTheta = kPTheta;
        m_name = Names.name(this);
    }

    @Override
    public void setTrajectory(final TrajectoryTimeIterator iter) {
        m_iter = iter;
        m_prevTimeS = Double.POSITIVE_INFINITY;
    }

    /** Makes no attempt to produce feasible output. */
    @Override
    public ChassisSpeeds update(double timeS, Pose2d measurement, Twist2d current_velocity) {
        if (m_iter == null)
            return null;

        t.log(Level.DEBUG, m_name, "measurement", measurement);
        if (isDone()) {
            return new ChassisSpeeds();
        }

        Optional<TimedPose> mSetpoint = getSetpoint(timeS);

        if (!mSetpoint.isPresent()) {
            return new ChassisSpeeds();
        }
        error = DriveMotionControllerUtil.getError(measurement, mSetpoint.get());
        t.log(Level.DEBUG, m_name, "setpoint", mSetpoint.get());
        t.log(Level.DEBUG, m_name, "error", error);

        ChassisSpeeds u_FF = DriveMotionControllerUtil.feedforward(measurement, mSetpoint.get());
        if (m_feedforwardOnly)
            return u_FF;
        ChassisSpeeds u_FB = DriveMotionControllerUtil.feedback(measurement, mSetpoint.get(), m_kPCart, m_kPTheta);
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
            return Optional.empty();
        }
        t.log(Level.DEBUG, m_name, "sample point", sample_point.get());
        return Optional.of(sample_point.get().state());
    }

    @Override
    public boolean isDone() {
        return m_iter != null && m_iter.isDone() && error.getTranslation().getNorm() < kTolerance ;
    }
}
