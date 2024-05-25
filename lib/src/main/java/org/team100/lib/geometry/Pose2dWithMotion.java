package org.team100.lib.geometry;

import java.text.DecimalFormat;
import java.util.Optional;

import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * This is from 254 2023 motion planning; the main reason to include it
 * is because it represents both heading and course, whereas the WPI equivalent
 * represents only course.
 * 
 * Note, Pose2dWithMotion is a purely spatial construct.
 */
public class Pose2dWithMotion {
    public static final Pose2dWithMotion kIdentity = new Pose2dWithMotion(
            GeometryUtil.kPoseZero, GeometryUtil.kTwist2dIdentity, 0, 0);

    private final Pose2d m_pose;

    /**
     * Even though this Twist provides scalar values for dx, dy, and dtheta,
     * Pose2dWithMotion is purely a spatial construct. It has no sense of time. So
     * rather than being in units of distance-per-time (or radians-per-time), the
     * denominator here is actually distance as well. Thus, it isn't really
     * meaningful to look at dx or dy directly - what is meaningful is:
     *
     * a) whether or not they are both zero (in which case this is a stationary or
     * turn-in-place motion)
     *
     * b) the angle formed by them, which is the direction of translation (in the
     * same coordinate frame as pose_).
     *
     * Additionally, this means dtheta is in radians-per-distance if there is
     * translation, or radians-per-radian otherwise.
     * 
     * TODO: eliminate the use of Twist2d here, make a specific type for this
     * concept.
     */
    private final Twist2d m_fieldRelativeMotionDirection;

    /**
     * Curvature is change in angle per change in distance, rad/m.
     */
    private final double m_curvatureRad_M;

    /**
     * DCurvatureDs is change in curvature per meter, rad/m^2
     */
    private final double m_dCurvatureDsRad_M2;

    /**
     * Motionless, no curvature.
     * 
     * @param pose
     */
    public Pose2dWithMotion(Pose2d pose) {
        this(pose, GeometryUtil.kTwist2dIdentity, 0, 0);
    }

    /**
     * 
     * @param pose                         Represents the location and heading of
     *                                     the robot
     * @param fieldRelativeMotionDirection Represents the change in location and
     *                                     heading, per meter traveled.
     * @param curvatureRad_M               Represents the change in course per meter
     *                                     traveled.
     * @param dCurvatureDsRad_M2           Represents the acceleration in course per
     *                                     meter traveled squared.
     */
    public Pose2dWithMotion(
            Pose2d pose,
            Twist2d fieldRelativeMotionDirection,
            double curvatureRad_M,
            double dCurvatureDsRad_M2) {
        m_pose = pose;
        m_fieldRelativeMotionDirection = fieldRelativeMotionDirection;
        m_curvatureRad_M = curvatureRad_M;
        m_dCurvatureDsRad_M2 = dCurvatureDsRad_M2;
    }

    public final Pose2d getPose() {
        return m_pose;
    }

    /** Radians per meter. */
    public double getCurvature() {
        return m_curvatureRad_M;
    }

    /** Radians per meter squared */
    public double getDCurvatureDs() {
        return m_dCurvatureDsRad_M2;
    }

    public final Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public final Rotation2d getHeading() {
        return getPose().getRotation();
    }

    // i think the interpolation of motion direction is invalid; it would yield
    // results not on the unit circle, which makes no sense. maybe the consumers of
    // motion direction always derive the angle anyway?
    public Pose2dWithMotion interpolate(final Pose2dWithMotion other, double x) {
        return new Pose2dWithMotion(getPose().interpolate(other.getPose(), x),
                GeometryUtil.interpolate(m_fieldRelativeMotionDirection, other.m_fieldRelativeMotionDirection, x),
                Math100.interpolate(getCurvature(), other.getCurvature(), x),
                Math100.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    /**
     * Distance in meters along the arc between the two poses (in either order)
     * produced by a constant twist.
     */
    public double distance(final Pose2dWithMotion other) {
        return GeometryUtil.distance(getPose(), other.getPose());
    }

    public boolean equals(final Object other) {
        if (!(other instanceof Pose2dWithMotion)) {
            return false;
        }

        Pose2dWithMotion p2dwc = (Pose2dWithMotion) other;
        return getPose().equals(p2dwc.getPose()) &&
                m_fieldRelativeMotionDirection.equals(p2dwc.m_fieldRelativeMotionDirection) &&
                Math100.epsilonEquals(getCurvature(), p2dwc.getCurvature()) &&
                Math100.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }

    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toString() + ", twist: " + m_fieldRelativeMotionDirection + ", curvature: "
                + fmt.format(getCurvature())
                + ", dcurvature_ds: " + fmt.format(getDCurvatureDs());
    }

    /**
     * Direction of the translational part of the motion, or empty if motionless.
     */
    public Optional<Rotation2d> getCourse() {
        return GeometryUtil.getCourse(m_fieldRelativeMotionDirection);
    }

    /**
     * Heading rate is radians per meter.
     * 
     * If you want radians per second, multiply by velocity (meters per second).
     */
    public double getHeadingRate() {
        return m_fieldRelativeMotionDirection.dtheta;
    }
}