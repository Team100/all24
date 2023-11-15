package org.team100.lib.motion.example2d.articulated_arm;

public class ArmActuation {
    private double m_proximalVelocityRad_S;
    private double m_distalVelocityRad_S;

    public ArmActuation(double proximalVelocityRad_S, double distalVelocityRad_S) {
        m_proximalVelocityRad_S = proximalVelocityRad_S;
        m_distalVelocityRad_S = distalVelocityRad_S;
    }

    public double getProximalVelocityRad_S() {
        return m_proximalVelocityRad_S;
    }

    public double getDistalVelocityRad_S() {
        return m_distalVelocityRad_S;
    }

}
