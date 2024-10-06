package org.team100.lib.profile;

import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.Profile100.ResultWithETA;
import org.team100.lib.state.State100;

/**
 * Coordinates three axes so that their profiles complete at about the same
 * time, by adjusting the maximum allowed acceleration.
 * 
 * Note that because acceleration is adjusted, but not cruise velocity, the
 * resulting paths will not be straight, for rest-to-rest profiles.
 */
public class HolonomicProfile {
    private static final double ETA_TOLERANCE = 0.02;

    private final double m_dt;
    private final TrapezoidProfile100 px;
    private final TrapezoidProfile100 py;
    private final TrapezoidProfile100 ptheta;

    private TrapezoidProfile100 ppx;
    private TrapezoidProfile100 ppy;
    private TrapezoidProfile100 pptheta;

    public HolonomicProfile(
            double dt,
            double maxXYVel,
            double maxXYAccel,
            double xyTolerance,
            double maxAngularVel,
            double maxAngularAccel,
            double angularTolerance) {
        m_dt = dt;
        px = new TrapezoidProfile100(maxXYVel, maxXYAccel, xyTolerance);
        py = new TrapezoidProfile100(maxXYVel, maxXYAccel, xyTolerance);
        ptheta = new TrapezoidProfile100(maxAngularVel, maxAngularAccel, angularTolerance);
    }

    /** Reset the scale factors. */
    public void solve(SwerveState i, SwerveState g) {
        // first find the max ETA
        ResultWithETA rx = px.calculateWithETA(m_dt, i.x(), g.x());
        ResultWithETA ry = py.calculateWithETA(m_dt, i.y(), g.y());
        ResultWithETA rtheta = ptheta.calculateWithETA(m_dt, i.theta(), g.theta());

        double slowETA = rx.etaS();
        slowETA = Math.max(slowETA, ry.etaS());
        slowETA = Math.max(slowETA, rtheta.etaS());

        double sx = px.solve(m_dt, i.x(), g.x(), slowETA, ETA_TOLERANCE);
        double sy = py.solve(m_dt, i.y(), g.y(), slowETA, ETA_TOLERANCE);
        double stheta = ptheta.solve(m_dt, i.theta(), g.theta(), slowETA, ETA_TOLERANCE);

        ppx = px.scale(sx);
        ppy = py.scale(sy);
        pptheta = ptheta.scale(stheta);
    }

    public SwerveState calculate(SwerveState i, SwerveState g) {
        State100 stateX = ppx.calculate(m_dt, i.x(), g.x());
        State100 stateY = ppy.calculate(m_dt, i.y(), g.y());
        State100 stateTheta = pptheta.calculate(m_dt, i.theta(), g.theta());
        return new SwerveState(stateX, stateY, stateTheta);
    }
}
