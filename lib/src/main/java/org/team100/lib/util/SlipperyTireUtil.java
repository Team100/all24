package org.team100.lib.util;

import org.team100.lib.copies.SwerveDriveKinematics100;
import org.team100.lib.geometry.Vector2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Utility functions for adjusting swerve module positions.
 * 
 * The general idea is to leave the kinematics alone, so that it operates on
 * "ideal" inputs that are actually "corner velocities" rather than wheel
 * speeds.
 * 
 * This class helps produce those corner velocities.
 */
public class SlipperyTireUtil {
    private final Tire m_tire;

    public SlipperyTireUtil(Tire tire) {
        m_tire = tire;
    }

    /**
     * @param corners current period extrapolated corner deltas
     * @param deltas  current period wheel deltas
     * @param dtS     current period time delta
     * @return adjusted corner deltas, suitable for forward kinematics
     */
    public SwerveModulePosition[] adjust(
            Vector2d[] corners,
            SwerveModulePosition[] deltas,
            double dtS) {
        if (corners.length != deltas.length)
            throw new IllegalArgumentException("delta length");
        SwerveModulePosition[] result = new SwerveModulePosition[deltas.length];
        for (int i = 0; i < deltas.length; i++) {
            SwerveModulePosition delta = deltas[i];
            Vector2d wheel = new Vector2d(
                    delta.distanceMeters * delta.angle.getCos(),
                    delta.distanceMeters * delta.angle.getSin());
            Vector2d actual = m_tire.actual(corners[i], wheel, dtS);
            result[i] = new SwerveModulePosition(
                    Math.hypot(actual.getX(), actual.getY()),
                    new Rotation2d(actual.getX(), actual.getY()));
        }
        return result;
    }

    /**
     * @param corners current period entry corner speeds
     * @param states  current period exit wheel speeds
     * @param dtS     length of the current period
     * @return adjusted corner speeds, suitable for forward kinematics
     */
    public SwerveModuleState[] adjust(
            Vector2d[] corners,
            SwerveModuleState[] states,
            double dtS) {
        if (corners.length != states.length)
            throw new IllegalArgumentException("delta length");
        SwerveModuleState[] result = new SwerveModuleState[states.length];
        for (int i = 0; i < states.length; i++) {
            SwerveModuleState state = states[i];
            Vector2d wheel = new Vector2d(
                    state.speedMetersPerSecond * state.angle.getCos(),
                    state.speedMetersPerSecond * state.angle.getSin());
            Vector2d actual = m_tire.actual(corners[i], wheel, dtS);
            result[i] = new SwerveModuleState(
                    Math.hypot(actual.getX(), actual.getY()),
                    new Rotation2d(actual.getX(), actual.getY()));
        }
        return result;
    }

    public static Vector2d[] cornerDeltas(
            SwerveDriveKinematics100 kinematics,
            Pose2d pose0,
            Pose2d pose1) {
        Twist2d twist = pose0.log(pose1);
        SwerveModulePosition[] p = kinematics.toSwerveModulePosition(twist);
        return kinematics.pos2vec(p);
    }
}
