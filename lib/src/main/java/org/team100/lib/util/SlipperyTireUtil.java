package org.team100.lib.util;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;

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
public class SlipperyTireUtil implements Glassy {
    /** Clip corner speeds to this. */
    private static final double kMaxSpeedM_s = 5.0;
    private final Logger m_logger;

    private final Tire m_tire;

    public SlipperyTireUtil(Logger parent, Tire tire) {
        m_tire = tire;
        m_logger = parent.child(this);
    }

    /**
     * Corner deltas are robot-relative.
     * 
     * This imposes a maximum speed to avoid over-responding to noise.
     * 
     * @param corners   current period extrapolated robot-relative corner deltas
     * @param cornerDtS time delta for corners
     * @param deltas    current period wheel deltas, meters
     * @param dtS       current period time delta, sec
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
            m_logger.logSwerveModulePosition(Level.TRACE, "deltas" + i, () -> delta);

            // this is robot-relative
            Vector2d wheelSpeedM_s = new Vector2d(
                    delta.distanceMeters * delta.angle.getCos() / dtS,
                    delta.distanceMeters * delta.angle.getSin() / dtS);

            // corners are robot-relative
            final Vector2d corner = corners[i];
            m_logger.logVector2d(Level.TRACE, "corner" + i, () -> corner);
            Vector2d cornerSpeedM_s = getCornerSpeedM_s(corners[i], cornerDtS);
            Vector2d actualSpeedM_s = m_tire.actual(cornerSpeedM_s, wheelSpeedM_s, dtS);
            m_logger.logVector2d(Level.TRACE, "cornerSpeed" + i, () -> cornerSpeedM_s);
            m_logger.logVector2d(Level.TRACE, "actualSpeed" + i, () -> actualSpeedM_s);

            // this throws away the "optimization" of the input. :(
            // TODO: fix that

            result[i] = new SwerveModulePosition(
                    Math.hypot(actualSpeedM_s.getX(), actualSpeedM_s.getY()) * dtS,
                    new Rotation2d(actualSpeedM_s.getX(), actualSpeedM_s.getY()));

            final SwerveModulePosition resulti = result[i];
            m_logger.logSwerveModulePosition(Level.TRACE, "result" + i, () -> resulti);
        }
        m_logger.logDouble(Level.TRACE, "dts", () -> dtS);
        return result;
    }

    private Vector2d getCornerSpeedM_s(Vector2d corner, double cornerDtS) {
        Vector2d cornerSpeedM_s = corner.times(1 / cornerDtS);
        // cap the allowed corner speed.
        if (cornerSpeedM_s.norm() > kMaxSpeedM_s) {
            cornerSpeedM_s = cornerSpeedM_s.times(kMaxSpeedM_s / cornerSpeedM_s.norm());
        }
        return cornerSpeedM_s;
    }

    /**
     * @param corners current period entry corner speeds
     * @param states  current period exit wheel speeds
     * @param dtS     length of the current period
     * @return adjusted corner speeds, suitable for forward kinematics
     */
    public SwerveModuleState100[] adjust(
            Vector2d[] corners,
            SwerveModuleState100[] states,
            double dtS) {
        if (corners.length != states.length)
            throw new IllegalArgumentException("delta length");
        SwerveModuleState100[] result = new SwerveModuleState100[states.length];
        for (int i = 0; i < states.length; i++) {
            SwerveModuleState100 state = states[i];
            Vector2d wheel = new Vector2d(
                    state.speedMetersPerSecond * state.angle.getCos(),
                    state.speedMetersPerSecond * state.angle.getSin());
            Vector2d actual = m_tire.actual(corners[i], wheel, dtS);
            result[i] = new SwerveModuleState100(
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
    public Vector2d[] cornerDeltas(
            SwerveDriveKinematics100 kinematics,
            Pose2d pose0,
            Pose2d pose1) {
        m_logger.logDouble(Level.TRACE, "pose0x", pose0::getX);
        m_logger.logDouble(Level.TRACE, "pose1x", pose1::getX);
        Twist2d twist = pose0.log(pose1);
        m_logger.logDouble(Level.TRACE, "twistdx", () -> twist.dx);
        SwerveModulePosition[] p = kinematics.toSwerveModulePosition(twist);
        return kinematics.pos2vec(p);
    }

    @Override
    public String getGlassName() {
        return "SlipperyTireUtil";
    }
}
