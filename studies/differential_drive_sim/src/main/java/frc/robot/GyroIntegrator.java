package frc.robot;

/** Integrate the gyro rate signal to obtain heading. */
public class GyroIntegrator {
    private double headingNWU = 0;

    public void addRateNWU(double rateDegSecNWU, double dt) {
        headingNWU += rateDegSecNWU * dt;

    }

    public double getHeadingNWU() {
        return headingNWU;
    }

    public void reset() {
        headingNWU = 0;
    }
}
