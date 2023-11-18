package org.team100.lib.motion.crank;

public class ActuationConstant implements Actuations {
    private final Actuation m_actuation;

    public ActuationConstant(Actuation actuation) {
        m_actuation = actuation;
    }

    @Override
    public Actuation get() {
        return m_actuation;
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }
    
}
