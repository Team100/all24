package org.team100.lib.config;

public class FeedforwardConstants {
    private final double m_kV;
    private final double m_kA;
    private final double m_kSS;
    private final double m_kDS;

    /**
    *Default no friction or attached objects contraints for NEO
    */
    public static FeedforwardConstants makeNeo() {
        return new FeedforwardConstants(0.122, 0, 0.1, 0.065);
    }

    public static FeedforwardConstants makeWCPSwerveTurningFalcon() {
        return new FeedforwardConstants(0.11, 0, 0.18, 0.01);
    }

    //TODO tune the static and dynamic friction
    public static FeedforwardConstants makeWCPSwerveTurningFalcon6() {
        return new FeedforwardConstants(0.16, 0, 0.08, 0.1);
    }
    
    public static FeedforwardConstants makeWCPSwerveDriveFalcon() {
        return new FeedforwardConstants(0.11, 0, 0.375, 0.27);
    }

    public static FeedforwardConstants makeWCPSwerveDriveFalcon6() {
        return new FeedforwardConstants(.13, .13, .374, .37);
        // return new FeedforwardConstants(0, 0, 0, 0);

    }

    public FeedforwardConstants() {
       m_kV = 0;
       m_kA = 0;
       m_kSS = 0;
       m_kDS = 0;
    }

    public FeedforwardConstants(double kV) {
        m_kV = kV;
        m_kA = 0;
        m_kSS = 0;
        m_kDS = 0;
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
