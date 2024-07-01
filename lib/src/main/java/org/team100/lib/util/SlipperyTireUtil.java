package org.team100.lib.util;

import org.team100.lib.geometry.Vector2d;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

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
    /** Clip corner speeds to this. */
    private static final double kMaxSpeedM_s = 5.0;
    private static final Telemetry t = Telemetry.get();

    private final Tire m_tire;

    public SlipperyTireUtil(Tire tire) {
        m_tire = tire;
    }

    /**
     * Corner deltas are robot-relative.
     * 
     * This imposes a maximum speed to avoid over-responding to noise.
     * 
     * @param corners   current period extrapolated robot-relative corner deltas
     * @param cornerDtS time delta for corners
     * @param deltas    current period wheel deltas, meters
     * @param dtS       current period time delta, meters
     * @return adjusted corner deltas, suitable for forward kinematics
     */
    public SwerveModulePosition[] adjust(
            Vector2d[] corners,
            double cornerDtS,
            SwerveModulePosition[] deltas,
            double dtS) {
        if (corners.length != deltas.length)
            throw new IllegalArgumentException("delta length");
        SwerveModulePosition[] result = new SwerveModulePosition[deltas.length];
        for (int i = 0; i < deltas.length; i++) {
            SwerveModulePosition delta = deltas[i];
            t.log(Level.WARN, "tireutil", "deltas", delta);

            // this is robot-relative
            Vector2d wheelSpeedM_s = new Vector2d(
                    delta.distanceMeters * delta.angle.getCos() / dtS,
                    delta.distanceMeters * delta.angle.getSin() / dtS);

            // corners are robot-relative
            t.log(Level.WARN, "tireutil", "corner", corners[i]);
            Vector2d cornerSpeedM_s = corners[i].times(1 / cornerDtS);
            // cap the allowed corner speed.
            if (cornerSpeedM_s.norm() > kMaxSpeedM_s) {
                cornerSpeedM_s = cornerSpeedM_s.times(kMaxSpeedM_s / cornerSpeedM_s.norm());
            }
            Vector2d actualSpeedM_s = m_tire.actual(cornerSpeedM_s, wheelSpeedM_s, dtS);
            t.log(Level.WARN, "tireutil", "cornerSpeed", cornerSpeedM_s);
            t.log(Level.WARN, "tireutil", "actualSpeed", actualSpeedM_s);

            // this throws away the "optimization" of the input. :(
            // TODO: fix that

            result[i] = new SwerveModulePosition(
                    Math.hypot(actualSpeedM_s.getX(), actualSpeedM_s.getY()) * dtS,
                    new Rotation2d(actualSpeedM_s.getX(), actualSpeedM_s.getY()));

            t.log(Level.WARN, "tireutil", "result", result[i]);
            t.log(Level.WARN, "tireutil", "dts", dtS);
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

    /**
     * Robot-relative corner deltas.
     * 
     * Note this can produce unrealistically large deltas, reacting to noise.
     */
    public static Vector2d[] cornerDeltas(
            SwerveDriveKinematics100 kinematics,
            Pose2d pose0,
            Pose2d pose1) {
        t.log(Level.TRACE, "tireutil", "pose0x", pose0.getX());
        t.log(Level.TRACE, "tireutil", "pose1x", pose1.getX());
        Twist2d twist = pose0.log(pose1);
        t.log(Level.TRACE, "tireutil", "twistdx", twist.dx);
        SwerveModulePosition[] p = kinematics.toSwerveModulePosition(twist);
        return kinematics.pos2vec(p);
    }
}
