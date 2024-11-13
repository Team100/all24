package org.team100.lib.localization;

import java.util.Objects;

import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * Represents an odometry record. The record contains the inputs provided as
 * well as the pose that was observed based on these inputs, as well as the
 * previous record and its inputs.
 * 
 * TODO: add velocity and acceleration.
 */
class InterpolationRecord implements Interpolatable<InterpolationRecord> {
    private final SwerveDriveKinematics100 m_kinematics;

    // The pose observed given the current sensor inputs and the previous pose.
    final SwerveModel m_state;

    // The current encoder readings.
    final SwerveModulePositions m_wheelPositions;

    /**
     * Constructs an Interpolation Record with the specified parameters.
     *
     * @param kinematics
     * @param state          The pose observed given the current sensor inputs and
     *                       the previous pose.
     * @param gyro           The current gyro angle.
     * @param gyroRateRad_S  current omega
     * @param wheelPositions The current encoder readings. Makes a copy.
     */
    InterpolationRecord(
            SwerveDriveKinematics100 kinematics,
            SwerveModel state,
            SwerveModulePositions wheelPositions) {
        m_kinematics = kinematics;
        m_state = state;
        m_wheelPositions = new SwerveModulePositions(wheelPositions);
    }

    /**
     * Return the "interpolated" record. This object is assumed to be the starting
     * position, or lower bound.
     * 
     * Interpolates the wheel positions.
     * Integrates wheel positions to find the interpolated pose.
     * Interpolates the velocity.
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
        SwerveModulePositions wheelLerp = new SwerveModulePositions(
                m_wheelPositions.frontLeft().interpolate(endValue.m_wheelPositions.frontLeft(), t),
                m_wheelPositions.frontRight().interpolate(endValue.m_wheelPositions.frontRight(), t),
                m_wheelPositions.rearLeft().interpolate(endValue.m_wheelPositions.rearLeft(), t),
                m_wheelPositions.rearRight().interpolate(endValue.m_wheelPositions.rearRight(), t));

        // Create a twist to represent the change based on the interpolated sensor
        // inputs.
        Twist2d twist = m_kinematics.toTwist2d(
                DriveUtil.modulePositionDelta(m_wheelPositions, wheelLerp));
        Pose2d pose = m_state.pose().exp(twist);

        // these lerps are wrong but maybe close enough
        FieldRelativeVelocity startVelocity = m_state.velocity();
        FieldRelativeVelocity endVelocity = endValue.m_state.velocity();
        FieldRelativeVelocity velocity = startVelocity.plus(endVelocity.minus(startVelocity).times(t));

        SwerveModel newState = new SwerveModel(pose, velocity);
        return new InterpolationRecord(m_kinematics, newState, wheelLerp);
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
        return Objects.equals(m_wheelPositions, rec.m_wheelPositions)
                && Objects.equals(m_state, rec.m_state);
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_wheelPositions, m_state);
    }

    @Override
    public String toString() {
        return "InterpolationRecord [m_poseMeters=" + m_state
                + ", m_wheelPositions=" + m_wheelPositions + "]";
    }

}