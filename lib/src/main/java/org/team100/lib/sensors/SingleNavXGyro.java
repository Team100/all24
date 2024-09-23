package org.team100.lib.sensors;

import org.team100.lib.async.Async;
import org.team100.lib.config.Identity;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.BooleanSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

/**
 * Single NavX over USB or SPI.
 * 
 * to work around SPI issues, implement the SPI reset in AHRS as suggested?
 * 
 * https://www.chiefdelphi.com/t/navx2-disconnecting-reconnecting-intermittently-not-browning-out/425487/39
 * 
 * try USB supplemental power for the NavX-MXP?
 * 
 * @deprecated because the NavX just doesn't work very well (our most recent
 *             navx-mxp broke soon after we got it), support is nonexistent.
 */
@Deprecated
public class SingleNavXGyro implements Gyro {

    /**
     * The java code in {@link edu.wpi.first.hal.SPIJNI.spiReadB} has to read and
     * parse every update. 200 Hz seems to make it do a lot of work (7.5%
     * self time according to VisualVM).
     */
    // private static final byte kUpdateRateHz = (byte) 200;

    /**
     * 60 Hz is the default, use that until we get a handle on the load issue.
     * 
     * Note that the actual update rate is 66
     * 
     * https://www.chiefdelphi.com/t/navx2-disconnecting-reconnecting-intermittently-not-browning-out/425487/39
     */
    // private static final byte kUpdateRateHz = (byte) 66;
    private static final byte kUpdateRateHz = (byte) 60;

    /**
     * RoboRIO SPI can go up to 4 MHz:
     * 
     * https://www.ni.com/docs/en-US/bundle/roborio-frc-specs/page/specs.html
     * 
     * The NavX SPI can go up to 2 Mhz:
     * 
     * https://pdocs.kauailabs.com/navx-mxp/advanced/register-protocol/
     * 
     * But each update is only something like 100 bytes, 1000 bits, 60 kHz, so
     * choose a lower bit rate:
     */
    private static final int kSPIBitRateHz = 500000;

    private final AHRS m_ahrs;

    /**
     * The NavX scale factor seems to not be set correctly from the factory:
     * 
     * https://www.chiefdelphi.com/t/navx-drift/467248/6
     * 
     * This should be measured for each NavX; it's configured here based on RoboRIO
     * identity, so remember to update this if you move NavX from one RoboRIO to
     * another.
     */
    private final float m_yawScaleFactor;
    private final float m_yawRateScaleFactor;
    // LOGGERS
    private final DoubleSupplierLogger2 m_log_heading;
    private final DoubleSupplierLogger2 m_log_heading_rate;
    private final DoubleSupplierLogger2 m_log_pitch;
    private final DoubleSupplierLogger2 m_log_roll;
    private final DoubleSupplierLogger2 m_log_yaw_deg;
    private final DoubleSupplierLogger2 m_log_pitch_deg;
    private final DoubleSupplierLogger2 m_log_roll_deg;
    private final DoubleSupplierLogger2 m_log_yaw_rate_deg;
    private final BooleanSupplierLogger2 m_log_connected;

    /**
     * NOTE: the async is just for logging, maybe don't use a whole thread for it.
     */
    public SingleNavXGyro(SupplierLogger2 parent, Async async) {
        SupplierLogger2 child = parent.child(this);

        // maximum update rate == minimum latency (use most-recent updates). maybe too
        // much CPU?
        switch (Identity.instance) {
            case COMP_BOT:
                m_ahrs = new AHRS(
                        SerialPort.Port.kUSB,
                        AHRS.SerialDataType.kProcessedData,
                        kUpdateRateHz);
                m_yawScaleFactor = 1.0f;
                m_yawRateScaleFactor = 1.0f;
                // remove this message when calibration is finished.
                Util.warn("********** NAVX SCALE FACTOR IS UNCALIBRATED!  CALIBRATE ME! **********");
                break;
            default:
                m_ahrs = new AHRS(
                        SPI.Port.kMXP,
                        kSPIBitRateHz,
                        kUpdateRateHz);
                m_yawScaleFactor = 1.0f;
                m_yawRateScaleFactor = 1.0f;
                // remove this message when calibration is finished.
                Util.warn("********** NAVX SCALE FACTOR IS UNCALIBRATED!  CALIBRATE ME! **********");
                break;

        }
        m_ahrs.enableBoardlevelYawReset(true);

        while (!m_ahrs.isConnected()) {
            Util.println("waiting for navx connection...");
            Timer.delay(1);
        }

        while ((m_ahrs.isConnected() && m_ahrs.isCalibrating())) {
            Util.println("Waiting for navx startup calibration...");
            Timer.delay(1);
        }

        m_ahrs.zeroYaw();
        async.addPeriodic(this::logStuff, 1, "SingleNavXGyro");
        m_log_heading = child.doubleLogger(Level.TRACE, "Heading NWU (rad)");
        m_log_heading_rate = child.doubleLogger(Level.TRACE, "Heading Rate NWU (rad_s)");
        m_log_pitch = child.doubleLogger(Level.TRACE, "Pitch NWU (rad)");
        m_log_roll = child.doubleLogger(Level.TRACE, "Roll NWU (rad)");
        m_log_yaw_deg = child.doubleLogger(Level.TRACE, "Yaw NED (deg)");
        m_log_pitch_deg = child.doubleLogger(Level.TRACE, "Pitch (deg)");
        m_log_roll_deg = child.doubleLogger(Level.TRACE, "Roll (deg)");
        m_log_yaw_rate_deg = child.doubleLogger(Level.TRACE, "Rate NED (deg_s)");
        m_log_connected = child.booleanLogger(Level.TRACE, "Connected");

    }

    @Override
    public Rotation2d getYawNWU() {
        Rotation2d currentHeadingNWU = Rotation2d.fromDegrees(-1.0 * getYawNEDDeg());
        m_log_heading.log(currentHeadingNWU::getRadians);
        return currentHeadingNWU;
    }

    @Override
    public double getYawRateNWU() {
        double currentHeadingRateNWU = Math.toRadians(getYawRateNEDDeg_s());
        m_log_heading_rate.log(() -> currentHeadingRateNWU);
        return currentHeadingRateNWU;
    }

    @Override
    public Rotation2d getPitchNWU() {
        Rotation2d pitchNWU = Rotation2d.fromDegrees(-1.0 * getPitchDeg());
        m_log_pitch.log(pitchNWU::getRadians);
        return pitchNWU;
    }

    @Override
    public Rotation2d getRollNWU() {
        Rotation2d rollNWU = Rotation2d.fromDegrees(-1.0 * getRollDeg());
        m_log_roll.log(rollNWU::getRadians);
        return rollNWU;
    }

    ///////////////////////
    //
    // below was previously in SingleNavXGyro.
    //

    /**
     * NOTE NOTE NOTE this is NED = clockwise positive = backwards
     * 
     * @returns yaw in degrees [-180,180]
     */
    private float getYawNEDDeg() {
        float yawDeg = m_ahrs.getYaw() * m_yawScaleFactor;
        m_log_yaw_deg.log(() -> yawDeg);
        return yawDeg;
    }

    /**
     * @returns pitch in degrees [-180,180]
     */
    private float getPitchDeg() {
        float pitchDeg = m_ahrs.getPitch();
        m_log_pitch_deg.log(() -> pitchDeg);
        return pitchDeg;
    }

    /**
     * @returns roll in degrees [-180,180]
     */
    private float getRollDeg() {
        float rollDeg = m_ahrs.getRoll();
        m_log_roll_deg.log(() -> rollDeg);
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
    private float getYawRateNEDDeg_s() {
        final float rateDeg_S = getRateDeg_S() * m_yawRateScaleFactor;
        m_log_yaw_rate_deg.log(() -> rateDeg_S);
        return rateDeg_S;
    }

    private float getRateDeg_S() {
        // 2/27/24 the NavX getRate() method has been broken since at least 2018
        //
        // https://github.com/kauailabs/navxmxp/issues/69
        //
        // the recommended workaround is to use getRawGyroZ() instead.
        return m_ahrs.getRawGyroZ();
        // float rateDeg_S = m_ahrs.getRawGyroZ();

        // NavX spec says the noise density of the gyro is 0.005 deg/s/sqrt(hz), and the
        // bandwidth is 6600 hz, so the expected noise is about 0.007 rad/s.
        // The zero offset is specified as 1 deg/s (0.02 rad/s).
        // The deadband here is very slow: 0.05 rad/s is 2 min/revolution
        // measurement here is degrees, 0.05 rad is about 2.9 deg
        //
        // use a filter here instead of a deadband.
        // if (Math.abs(rateDeg_S) < 2.9) {
        // rateDeg_S = 0;
        // }
        // return rateDeg_S;
    }

    private void logStuff() {
        if (m_ahrs.isConnected()) {
            m_log_connected.log(() -> true);
        } else {
            m_log_connected.log(() -> false);
        }
        m_log_yaw_deg.log(m_ahrs::getYaw);
    }

}
