package org.team100.lib.sensors;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class AHRS {
    private final LSM6DSOX_I2C m_gyro;
    private double headingNWURad;
    private double previousTimeSec;

    private final DoublePublisher headingPub;
    private final DoublePublisher ratePub;

    public AHRS(LSM6DSOX_I2C gyro) {
        m_gyro = gyro;
        previousTimeSec = Timer.getFPGATimestamp();
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("gyro");
        headingPub = table.getDoubleTopic("headingNWURad").publish();
        ratePub = table.getDoubleTopic("rateNWURadS").publish();
    }

    public double getHeadingNWURad() {
        return headingNWURad;
    }

    /** Call this as often as possible. */
    public void update() {
        double newTimeSec = Timer.getFPGATimestamp();
        double dtSec = newTimeSec - previousTimeSec;
        previousTimeSec = newTimeSec;
        double yawRateRadS = m_gyro.getYawRateRadS();
        ratePub.set(yawRateRadS);
        double yawDeltaRad = yawRateRadS * dtSec;
        headingNWURad += yawDeltaRad;
        headingPub.set(headingNWURad);
    }

    public void reset() {
        headingNWURad = 0;
    }
}
