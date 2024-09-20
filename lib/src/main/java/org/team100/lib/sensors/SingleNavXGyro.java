package org.team100.lib.sensors;

import org.team100.lib.async.Async;
import org.team100.lib.config.Identity;
import org.team100.lib.logging.SupplierLogger2;
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
 * TODO: to work around SPI issues, implement the SPI reset in AHRS as suggested
 * 
 * https://www.chiefdelphi.com/t/navx2-disconnecting-reconnecting-intermittently-not-browning-out/425487/39
 * 
 * TODO: try USB supplemental power for the NavX-MXP
 */
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

    private final SupplierLogger2 m_logger;

    // TODO: remove this if it's not useful
    // private final AHRS100 m_ahrs;
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

    /**
     * NOTE: the async is just for logging, maybe don't use a whole thread for it.
     */
    public SingleNavXGyro(SupplierLogger2 parent, Async async) {

        m_logger = parent.child(this);

        // maximum update rate == minimum latency (use most-recent updates). maybe too
        // much CPU?
        switch (Identity.instance) {
            // case COMP_BOT:
            default:
                // Jun 29 2024: actually use the specified bit rate
                // m_gyro1 = new AHRS(SPI.Port.kMXP);
                // this is the version i hacked to avoid wpilib 2025 breaking changes
                // m_ahrs = new AHRS100(SPI.Port.kMXP, kSPIBitRateHz, kUpdateRateHz);
                m_ahrs = new AHRS(
                        SPI.Port.kMXP,
                        kSPIBitRateHz,
                        kUpdateRateHz);
                m_yawScaleFactor = 1.0f;
                m_yawRateScaleFactor = 1.0f;
                // TODO: remove this message when calibration is finished.
                Util.warn("********** NAVX SCALE FACTOR IS UNCALIBRATED!  CALIBRATE ME! **********");
                break;
            // default:
            case COMP_BOT:
                // this is the version i hacked to avoid wpilib 2025 breaking changes
                // m_ahrs = new AHRS100(SerialPort.Port.kUSB,
                // AHRS100.SerialDataType.kProcessedData, kUpdateRateHz);
                m_ahrs = new AHRS(
                        SerialPort.Port.kUSB,
                        AHRS.SerialDataType.kProcessedData,
                        kUpdateRateHz);
                m_yawScaleFactor = 1.0f;
                m_yawRateScaleFactor = 1.0f;
                // TODO: remove this message when calibration is finished.
                Util.warn("********** NAVX SCALE FACTOR IS UNCALIBRATED!  CALIBRATE ME! **********");

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

    }

    @Override
    public Rotation2d getYawNWU() {
        Rotation2d currentHeadingNWU = Rotation2d.fromDegrees(-1.0 * getYawNEDDeg());
        m_logger.logDouble(Level.TRACE, "Heading NWU (rad)", currentHeadingNWU::getRadians);
        return currentHeadingNWU;
    }

    @Override
    public double getYawRateNWU() {
        double currentHeadingRateNWU = Math.toRadians(getYawRateNEDDeg_s());
        m_logger.logDouble(Level.TRACE, "Heading Rate NWU (rad_s)", () -> currentHeadingRateNWU);
        return currentHeadingRateNWU;
    }

    @Override
    public Rotation2d getPitchNWU() {
        Rotation2d pitchNWU = Rotation2d.fromDegrees(-1.0 * getPitchDeg());
        m_logger.logDouble(Level.TRACE, "Pitch NWU (rad)", pitchNWU::getRadians);
        return pitchNWU;
    }

    @Override
    public Rotation2d getRollNWU() {
        Rotation2d rollNWU = Rotation2d.fromDegrees(-1.0 * getRollDeg());
        m_logger.logDouble(Level.TRACE, "Pitch NWU (rad)", rollNWU::getRadians);
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
        m_logger.logDouble(Level.TRACE, "Yaw NED (deg)", () -> yawDeg);
        return yawDeg;
    }

    /**
     * @returns pitch in degrees [-180,180]
     */
    private float getPitchDeg() {
        float pitchDeg = m_ahrs.getPitch();
        m_logger.logDouble(Level.TRACE, "Pitch (deg)", () -> pitchDeg);
        return pitchDeg;
    }

    /**
     * @returns roll in degrees [-180,180]
     */
    private float getRollDeg() {
        float rollDeg = m_ahrs.getRoll();
        m_logger.logDouble(Level.TRACE, "Roll (deg)", () -> rollDeg);
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
        m_logger.logDouble(Level.TRACE, "Rate NED (deg_s)", () -> rateDeg_S);
        return rateDeg_S;
    }

    private float getRateDeg_S() {
        // 2/27/24 the NavX getRate() method has been broken since at least 2018
        //
        // https://github.com/kauailabs/navxmxp/issues/69
        //
        // the recommended workaround is to use getRawGyroZ() instead.
        float rateDeg_S = m_ahrs.getRawGyroZ();

        // NavX spec says the noise density of the gyro is 0.005 deg/s/sqrt(hz), and the
        // bandwidth is 6600 hz, so the expected noise is about 0.007 rad/s.
        // The zero offset is specified as 1 deg/s (0.02 rad/s).
        // The deadband here is very slow: 0.05 rad/s is 2 min/revolution
        // measurement here is degrees, 0.05 rad is about 2.9 deg
        // TODO: use a filter instead of a deadband.
        // if (Math.abs(rateDeg_S) < 2.9) {
        // rateDeg_S = 0;
        // }
        return rateDeg_S;
    }

    private void logStuff() {
        if (m_ahrs.isConnected()) {
            m_logger.logBoolean(Level.TRACE, "Connected", () -> true);
        } else {
            m_logger.logBoolean(Level.COMP, "Connected", () -> false);
        }
        m_logger.logDouble(Level.TRACE, "Yaw (deg)", m_ahrs::getYaw);
        // note we don't actually use any of the measurements below. maybe remove them?
        m_logger.logDouble(Level.TRACE, "Angle (deg)", m_ahrs::getAngle);
        m_logger.logDouble(Level.TRACE, "Angle Mod 360 (deg)", () -> m_ahrs.getAngle() % 360);
    }

}
