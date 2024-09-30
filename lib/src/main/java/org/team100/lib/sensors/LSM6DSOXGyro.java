package org.team100.lib.sensors;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Wraps the LSM6DSOX class in the Gyro interface.
 * 
 * Note this should be used as a sort of last resort; the MXP I2C may or may not
 * work.
 * 
 * TODO: extrapolate to the current instant
 */
public class LSM6DSOXGyro implements Gyro {

    private final LSM6DSOX_I2C m_gyro;
    private double headingNWURad;
    private Double prevRateRad_S = null;
    private double previousTimeSec;

    public LSM6DSOXGyro() {
        m_gyro = new LSM6DSOX_I2C();
        previousTimeSec = Timer.getFPGATimestamp();
    }

    /** mirrors real_gyro.py */
    @Override
    public Rotation2d getYawNWU() {
        double yawRateRadS = getYawRateNWU();
        if (prevRateRad_S == null) {
            prevRateRad_S = yawRateRadS;
        }
        double endTimeS = Timer.getFPGATimestamp();
        double durationS = endTimeS - previousTimeSec;
        previousTimeSec = endTimeS;
        // use the midpoint rule Riemann sum
        // https://en.wikipedia.org/wiki/Riemann_sum#Midpoint_rule
        double midRateRad_S = 0.5 * (yawRateRadS + prevRateRad_S);
        double dYawRad = midRateRad_S * durationS;
        headingNWURad += dYawRad;
        return new Rotation2d(headingNWURad);
    }

    @Override
    public double getYawRateNWU() {
        return m_gyro.getYawRateRadS();
    }

    @Override
    public Rotation2d getPitchNWU() {
        return GeometryUtil.kRotationZero;
    }

    @Override
    public Rotation2d getRollNWU() {
        return GeometryUtil.kRotationZero;
    }

    @Override
    public void periodic() {
        //
    }
}
