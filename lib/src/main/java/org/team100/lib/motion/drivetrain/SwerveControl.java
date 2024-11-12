package org.team100.lib.motion.drivetrain;

import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.Control100;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Describes the state of a holonomic drive in three dimensions,
 * x, y, and theta, each of which is represented by position, velocity,
 * and acceleration.
 * 
 * This type is used for control, which is why it includes acceleration
 */
public class SwerveControl {
    private final Control100 m_x;
    private final Control100 m_y;
    private final Control100 m_theta;

    public SwerveControl(Control100 x, Control100 y, Control100 theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    public SwerveControl(Pose2d x, FieldRelativeVelocity v) {
        this(
                new Control100(x.getX(), v.x(), 0),
                new Control100(x.getY(), v.y(), 0),
                new Control100(x.getRotation().getRadians(), v.theta(), 0));
    }

    public SwerveControl(Pose2d x, FieldRelativeVelocity v, FieldRelativeAcceleration a) {
        this(
                new Control100(x.getX(), v.x(), a.x()),
                new Control100(x.getY(), v.y(), a.y()),
                new Control100(x.getRotation().getRadians(), v.theta(), a.theta()));
    }

    public SwerveControl(Pose2d x) {
        this(x, new FieldRelativeVelocity(0, 0, 0));
    }

    public SwerveControl(Rotation2d x) {
        this(new Pose2d(0, 0, x));
    }

    public SwerveControl() {
        this(new Control100(), new Control100(), new Control100());
    }

    public SwerveControl withTheta(double theta) {
        return new SwerveControl(m_x, m_y, new Control100(theta, m_theta.v(), m_theta.a()));
    }

    public SwerveControl minus(SwerveControl other) {
        return new SwerveControl(x().minus(other.x()), y().minus(other.y()), theta().minus(other.theta()));
    }

    public SwerveControl plus(SwerveControl other) {
        return new SwerveControl(x().plus(other.x()), y().plus(other.y()), theta().plus(other.theta()));
    }

    public boolean near(SwerveControl other, double tolerance) {
        return x().near(other.x(), tolerance)
                && y().near(other.y(), tolerance)
                && theta().near(other.theta(), tolerance);
    }

    public Pose2d pose() {
        return new Pose2d(m_x.x(), m_y.x(), new Rotation2d(m_theta.x()));
    }

    /** Translation of the pose */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    public FieldRelativeVelocity velocity() {
        return new FieldRelativeVelocity(m_x.v(), m_y.v(), m_theta.v());
    }

    /** Robot-relative speeds */
    public ChassisSpeeds chassisSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                m_x.v(), m_y.v(), m_theta.v(), new Rotation2d(m_theta.x()));
    }

    public FieldRelativeAcceleration acceleration() {
        return new FieldRelativeAcceleration(m_x.a(), m_y.a(), m_theta.a());
    }

    public Control100 x() {
        return m_x;
    }

    public Control100 y() {
        return m_y;
    }

    public Control100 theta() {
        return m_theta;
    }

    /**
     * Transform timed pose into swerve state.
     * 
     * Correctly accounts for centripetal acceleration.
     */
    public static SwerveControl fromTimedPose(TimedPose timedPose) {
        double xx = timedPose.state().getPose().getX();
        double yx = timedPose.state().getPose().getY();
        double thetax = timedPose.state().getHeading().getRadians();

        double velocityM_s = timedPose.velocityM_S();
        Optional<Rotation2d> course = timedPose.state().getCourse();
        Rotation2d motion_direction = course.isPresent() ? course.get() : GeometryUtil.kRotationZero;
        double xv = motion_direction.getCos() * velocityM_s;
        double yv = motion_direction.getSin() * velocityM_s;
        double thetav = timedPose.state().getHeadingRate() * velocityM_s;

        double accelM_s_s = timedPose.acceleration();
        double xa = motion_direction.getCos() * accelM_s_s;
        double ya = motion_direction.getSin() * accelM_s_s;
        double thetaa = timedPose.state().getHeadingRate() * accelM_s_s;

        // centripetal accel = v^2/r = v^2 * curvature
        double curvRad_M = timedPose.state().getCurvature();
        double centripetalAccelM_s_s = velocityM_s * velocityM_s * curvRad_M;
        double xCa = -1.0 * motion_direction.getSin() * centripetalAccelM_s_s;
        double yCa = motion_direction.getCos() * centripetalAccelM_s_s;

        return new SwerveControl(
                new Control100(xx, xv, xa + xCa),
                new Control100(yx, yv, ya + yCa),
                new Control100(thetax, thetav, thetaa));
    }

    public String toString() {
        return "SwerveControl(" + m_x + ", " + m_y + ", " + m_theta + ")";
    }

}