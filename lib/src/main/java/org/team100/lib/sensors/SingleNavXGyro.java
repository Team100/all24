package org.team100.lib.sensors;

import org.team100.lib.async.Async;
import org.team100.lib.config.Identity;
import org.team100.lib.sensors.navx.AHRS100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

/**
 * Single NavX over USB, using high update rate.
 */
public class SingleNavXGyro implements Gyro100 {
    // 200 hz seems to make edu.wpi.first.hal.SPIJNI.spiReadB do a lot of work (7.5%
    // self time according to VisualVM)
    // private static final byte kUpdateRateHz = (byte) 200;
    // 60 is the default
    private static final byte kUpdateRateHz = (byte) 60;

    private static final int kSPIBitRateHz = 500000;
    private final Logger m_logger;
    // TODO: remove this if it's not useful
    // private final AHRS100 m_gyro1;
    private final AHRS m_gyro1;

    /**
     * NOTE: the async is just for logging, maybe don't use a whole thread for it.
     */
    public SingleNavXGyro(Logger parent, Async async) {
        m_logger = parent.child(this);

        // maximum update rate == minimum latency (use most-recent updates). maybe too
        // much CPU?
        switch (Identity.instance) {
            case COMP_BOT:
                // Jun 29 2024: actually use the specified bit rate
                // m_gyro1 = new AHRS(SPI.Port.kMXP);
                // this is the version i hacked to avoid wpilib 2025 breaking changes
                // m_gyro1 = new AHRS100(SPI.Port.kMXP, kSPIBitRateHz, kUpdateRateHz);
                m_gyro1 = new AHRS(SPI.Port.kMXP, kSPIBitRateHz, kUpdateRateHz);
                break;
            default:
                // this is the version i hacked to avoid wpilib 2025 breaking changes
                // m_gyro1 = new AHRS100(SerialPort.Port.kUSB, AHRS100.SerialDataType.kProcessedData, kUpdateRateHz);
                m_gyro1 = new AHRS(SerialPort.Port.kUSB, AHRS100.SerialDataType.kProcessedData, kUpdateRateHz);
        }
        m_gyro1.enableBoardlevelYawReset(true);

        Util.println("waiting for navx connection...");
        Timer.delay(2);

        while ((m_gyro1.isConnected() && m_gyro1.isCalibrating())) {
            Timer.delay(0.5);
            Util.println("Waiting for navx startup calibration...");
        }

        m_gyro1.zeroYaw();
        async.addPeriodic(this::logStuff, 1, "SingleNavXGyro");
    }

    /**
     * NOTE NOTE NOTE this is NED = clockwise positive = backwards
     * 
     * @returns yaw in degrees [-180,180]
     */
    @Override
    public float getYawNEDDeg() {
        float yawDeg = m_gyro1.getYaw();
        m_logger.logFloat(Level.TRACE, "Yaw NED (deg)", () -> yawDeg);
        return yawDeg;
    }

    /**
     * @returns pitch in degrees [-180,180]
     */
    @Override
    public float getPitchDeg() {
        float pitchDeg = m_gyro1.getPitch();
        m_logger.logFloat(Level.TRACE, "Pitch (deg)", () -> pitchDeg);
        return pitchDeg;
    }

    /**
     * @returns roll in degrees [-180,180]
     */
    @Override
    public float getRollDeg() {
        float rollDeg = m_gyro1.getRoll();
        m_logger.logFloat(Level.TRACE, "Roll (deg)", () -> rollDeg);
        return rollDeg;
    }

    /**
     * NOTE NOTE NOTE this is NED = clockwise positive = backwards
     * 
     * 6/12/24
     * WARNNINGGGGG DO NOT USE THIS WITHOUT AN MXP GYRO, IT WILL ALWAYS RETURN 0
     * 
     * @returns rate in degrees/sec
     */
    @Override
    public float getYawRateNEDDeg_s() {
        final double rateDeg_S = getRateDeg_S();
        m_logger.logDouble(Level.TRACE, "Rate NED (deg_s)", () -> rateDeg_S);
        return (float) rateDeg_S;
    }

    private double getRateDeg_S() {
        // 2/27/24 the NavX getRate() method has been broken since at least 2018
        //
        // https://github.com/kauailabs/navxmxp/issues/69
        //
        // the recommended workaround is to use getRawGyroZ() instead.
        double rateDeg_S = m_gyro1.getRawGyroZ();

        // NavX spec says the noise density of the gyro is 0.005 deg/s/sqrt(hz), and the
        // bandwidth is 6600 hz, so the expected noise is about 0.007 rad/s.
        // The zero offset is specified as 1 deg/s (0.02 rad/s).
        // The deadband here is very slow: 0.05 rad/s is 2 min/revolution
        // measurement here is degrees, 0.05 rad is about 2.9 deg
        if (Math.abs(rateDeg_S) < 2.9) {
            rateDeg_S = 0;
        }
        return rateDeg_S;
    }

    private void logStuff() {
        if (m_gyro1.isConnected()) {
            m_logger.logBoolean(Level.TRACE, "Connected", () -> true);
        } else {
            m_logger.logBoolean(Level.COMP, "Connected", () -> false);
        }
        m_logger.logDouble(Level.TRACE, "Angle (deg)", m_gyro1::getAngle);
        m_logger.logFloat(Level.TRACE, "Fused (deg)", m_gyro1::getFusedHeading);
        m_logger.logFloat(Level.TRACE, "Yaw (deg)", m_gyro1::getYaw);
        m_logger.logDouble(Level.TRACE, "Angle Mod 360 (deg)", () -> m_gyro1.getAngle() % 360);
        m_logger.logFloat(Level.TRACE, "Compass Heading (deg)", m_gyro1::getCompassHeading);
    }
}
