package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;

/**
 * Enforce workspace feasibility by editing reference state.
 */
public class WorkspaceFeasible implements Workstates {
    private final Supplier<Workstates> m_follower;
    private final double m_maxVelocityM_S;
    private final double m_maxAccelM_S_S;
    private double m_prevVelM_S;
    private double m_prevTimeS;

    /**
     * @param follower is a supplier of followers so that it can be mutable.
     */
    public WorkspaceFeasible(
            Supplier<Workstates> follower,
            double maxVelocityM_S,
            double maxAccelM_S_S) {
        m_follower = follower;
        m_maxVelocityM_S = maxVelocityM_S;
        m_maxAccelM_S_S = maxAccelM_S_S;
        m_prevTimeS = 0;
        m_prevVelM_S = 0;
    }

    @Override
    public Workstate get() {
        Workstate velocityM_S = m_follower.get().get();

        double nowS = Timer.getFPGATimestamp();
        double dtS = nowS - m_prevTimeS;
        m_prevTimeS = nowS;
        double accelM_S_S = (velocityM_S.getWorkstate().getState() - m_prevVelM_S) / dtS;
        if (velocityM_S.getWorkstate().getState() > m_maxVelocityM_S) {
            return new Workstate(m_maxAccelM_S_S);
        }
        if (velocityM_S.getWorkstate().getState() < -m_maxVelocityM_S) {
            return new Workstate(-m_maxVelocityM_S);
        }
        if (accelM_S_S > m_maxAccelM_S_S) {
            return new Workstate(m_prevVelM_S + m_maxAccelM_S_S * dtS);
        }
        if (accelM_S_S < -m_maxAccelM_S_S) {
            return new Workstate(m_prevVelM_S - m_maxAccelM_S_S * dtS);
        }
        return velocityM_S;
    }

    @Override
    public void accept(Indicator indicator) {
        m_follower.get().accept(indicator);
        indicator.indicate(this);
    }
}
