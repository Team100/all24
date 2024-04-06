package org.team100.lib.copies;

import java.util.Objects;

import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;

/**
 * Represents an odometry record. The record contains the inputs provided as
 * well as the pose that was observed based on these inputs, as well as the
 * previous record and its inputs.
 */
class InterpolationRecord implements Interpolatable<InterpolationRecord> {
    private final SwerveDriveKinematics100 m_kinematics;

    // The pose observed given the current sensor inputs and the previous pose.
    final Pose2d m_poseMeters;

    // The current gyro angle.
    final Rotation2d m_gyroAngle;

    // The current encoder readings.
    final SwerveDriveWheelPositions m_wheelPositions;

    /**
     * Constructs an Interpolation Record with the specified parameters.
     *
     * @param poseMeters     The pose observed given the current sensor inputs and
     *                       the previous pose.
     * @param gyro           The current gyro angle.
     * @param wheelPositions The current encoder readings.
     */
    InterpolationRecord(
            SwerveDriveKinematics100 kinematics,
            Pose2d poseMeters,
            Rotation2d gyro,
            SwerveDriveWheelPositions wheelPositions) {
        m_kinematics = kinematics;
        m_poseMeters = poseMeters;
        m_gyroAngle = gyro;
        m_wheelPositions = wheelPositions;
    }

    /**
     * Return the interpolated record. This object is assumed to be the starting
     * position, or lower bound.
     *
     * @param endValue The upper bound, or end.
     * @param t        How far between the lower and upper bound we are. This should
     *                 be bounded in [0, 1].
     * @return The interpolated value.
     */
    @Override
    public InterpolationRecord interpolate(InterpolationRecord endValue, double t) {
        if (t < 0) {
            return this;
        }
        if (t >= 1) {
            return endValue;
        }
        // Find the new wheel distances.
        SwerveDriveWheelPositions wheelLerp = m_wheelPositions.interpolate(endValue.m_wheelPositions, t);

        // Find the new gyro angle.
        Rotation2d gyroLerp = m_gyroAngle.interpolate(endValue.m_gyroAngle, t);

        // Create a twist to represent the change based on the interpolated sensor
        // inputs.
        // TODO: this should take tires into account since it modifies the pose
        // estimate.
        Twist2d twist = m_kinematics.toTwist2d(
                DriveUtil.modulePositionDelta(m_wheelPositions, wheelLerp));
        twist.dtheta = gyroLerp.minus(m_gyroAngle).getRadians();

        return new InterpolationRecord(m_kinematics, m_poseMeters.exp(twist), gyroLerp, wheelLerp);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof InterpolationRecord)) {
            return false;
        }
        InterpolationRecord rec = (InterpolationRecord) obj;
        return Objects.equals(m_gyroAngle, rec.m_gyroAngle)
                && Objects.equals(m_wheelPositions, rec.m_wheelPositions)
                && Objects.equals(m_poseMeters, rec.m_poseMeters);
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_gyroAngle, m_wheelPositions, m_poseMeters);
    }
}