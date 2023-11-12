package org.team100.lib.motion.example1d;

import java.util.function.DoubleUnaryOperator;
import java.util.function.UnaryOperator;

import org.team100.lib.motion.example1d.framework.Workstate;

import edu.wpi.first.wpilibj.Timer;

/** Workspace feasibility
 * TODO: add a parameter
 */
public class FeasibleFilter implements UnaryOperator<Workstate<Double>> {

    private final double m_maxVelocityM_S;
    private final double m_maxAccelM_S_S;
    private double m_prevVelM_S;
    private double m_prevTimeS;

    public FeasibleFilter( double maxVelocityM_S, double maxAccelM_S_S) {
        m_maxVelocityM_S = maxVelocityM_S;
        m_maxAccelM_S_S = maxAccelM_S_S;
        m_prevTimeS = 0;
        m_prevVelM_S = 0;
    }

    /**
     * Limit velocity and acceleration.
     * 
     * @return a feasible velocity in meters per second
     */
    @Override
    public Workstate<Double> apply(Workstate<Double> velocityM_S) {
        double nowS = Timer.getFPGATimestamp();
        double dtS = nowS - m_prevTimeS;
        m_prevTimeS = nowS;
        double accelM_S_S = (velocityM_S.getWorkstate() - m_prevVelM_S)/dtS;
        if (velocityM_S.getWorkstate() > m_maxVelocityM_S) {
            return new CrankWorkstate(m_maxAccelM_S_S);
        }
        if (velocityM_S.getWorkstate() < -m_maxVelocityM_S) {
            return new CrankWorkstate(-m_maxVelocityM_S);
        }
        if (accelM_S_S > m_maxAccelM_S_S) {
            return new CrankWorkstate(m_prevVelM_S + m_maxAccelM_S_S * dtS);
        }
        if (accelM_S_S < -m_maxAccelM_S_S) {
            return new CrankWorkstate(m_prevVelM_S - m_maxAccelM_S_S * dtS);
        }
        return velocityM_S;
    }
}
