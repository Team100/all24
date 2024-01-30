package org.team100.lib.config;

public class FeedforwardConstants {
    private final double m_kV;
    private final double m_kA;
    private final double m_kSS;
    private final double m_kDS;

    //Default no friction or attached objects contraints for NEO
    public FeedforwardConstants() {
       m_kV = 0.122;
       m_kA = 0;
       m_kSS = 0.1;
       m_kDS = 0.065;
    }

    public FeedforwardConstants(double kV, double kA, double kSS, double kDS) {
       m_kV = kV;
       m_kA = kA;
       m_kSS = kSS;
       m_kDS = kDS;
    }

    public double getkV(){
        return m_kV;
    }

    public double getkA(){
        return m_kA;
    }

    public double getkSS(){
        return m_kSS;
    }

    public double getkDS(){
        return m_kDS;
    }
}
