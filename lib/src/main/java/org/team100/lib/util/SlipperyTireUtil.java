package org.team100.lib.util;

import java.util.Optional;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

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
    public SwerveModulePosition100[] adjust(
            Vector2d[] corners,
            double cornerDtS,
            SwerveModulePosition100[] deltas,
            double dtS) {
        if (corners.length != deltas.length)
            throw new IllegalArgumentException("delta length");
        SwerveModulePosition100[] result = new SwerveModulePosition100[deltas.length];
        for (int i = 0; i < deltas.length; i++) {
            SwerveModulePosition100 delta = deltas[i];
            m_logger.logSwerveModulePosition100(Level.TRACE, "deltas" + i, () -> delta);

            // corners are robot-relative
            final Vector2d corner = corners[i];
            m_logger.logVector2d(Level.TRACE, "corner" + i, () -> corner);
            Vector2d cornerSpeedM_s = getCornerSpeedM_s(corners[i], cornerDtS);

            if (Math.abs(delta.distanceMeters) < 1e-6 || delta.angle.isEmpty()) {
                result[i] = safePosition(dtS, delta, m_tire.actual(cornerSpeedM_s, new Vector2d(0, 0), dtS));
                continue;
            }

            // this is robot-relative
            Vector2d wheelSpeedM_s = new Vector2d(
                    delta.distanceMeters * delta.angle.get().getCos() / dtS,
                    delta.distanceMeters * delta.angle.get().getSin() / dtS);

            Vector2d actualSpeedM_s = m_tire.actual(cornerSpeedM_s, wheelSpeedM_s, dtS);
            m_logger.logVector2d(Level.TRACE, "cornerSpeed" + i, () -> cornerSpeedM_s);
            m_logger.logVector2d(Level.TRACE, "actualSpeed" + i, () -> actualSpeedM_s);

            // this throws away the "optimization" of the input. :(
            // TODO: fix that

            result[i] = safePosition(dtS, delta, actualSpeedM_s);
            final SwerveModulePosition100 resulti = result[i];
            m_logger.logSwerveModulePosition100(Level.TRACE, "result" + i, () -> resulti);
        }
        m_logger.logDouble(Level.TRACE, "dts", () -> dtS);
        return result;
    }

    /** avoid invalid rotation. */
    private SwerveModulePosition100 safePosition(double dtS, SwerveModulePosition100 delta,
            Vector2d actualSpeedM_s) {
        double x = actualSpeedM_s.getX();
        double y = actualSpeedM_s.getY();
        if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6) {
            // the rotation below will be garbage so give up
            return delta;
        } else {
            return new SwerveModulePosition100(
                    actualSpeedM_s.norm() * dtS,
                    Optional.of(new Rotation2d(x, y)));
        }
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
            if (Math.abs(state.speedMetersPerSecond) < 1e-6 || state.angle.isEmpty()) {
                result[i] = safeState(state, m_tire.actual(corners[i], new Vector2d(0, 0), dtS));
                continue;
            }
            Vector2d wheel = new Vector2d(
                    state.speedMetersPerSecond * state.angle.get().getCos(),
                    state.speedMetersPerSecond * state.angle.get().getSin());
            Vector2d actual = m_tire.actual(corners[i], wheel, dtS);
            result[i] = safeState(state, actual);
        }
        return result;
    }

    /** avoid invalid rotation */
    private SwerveModuleState100 safeState(SwerveModuleState100 state, Vector2d actual) {
        if (actual.norm() < 1e-6) {
            // the rotation2d below will be garbage so give up
            return state;
        } else {
            return new SwerveModuleState100(
                    Math.hypot(actual.getX(), actual.getY()),
                    Optional.of(new Rotation2d(actual.getX(), actual.getY())));
        }
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
        SwerveModulePosition100[] p = kinematics.toSwerveModulePosition(twist);
        return kinematics.pos2vec(p);
    }

    @Override
    public String getGlassName() {
        return "SlipperyTireUtil";
    }
}
