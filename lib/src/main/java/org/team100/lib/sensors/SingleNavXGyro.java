package org.team100.lib.sensors;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

/**
 * Single NavX over USB, using high update rate.
 */
public class SingleNavXGyro implements Gyro100 {
    private static final byte kUpdateRateHz = (byte) 200;
    private final Telemetry t = Telemetry.get();
    private final AHRS m_gyro1;
    private final Notifier periodicLogger;
    private final String m_name;

    public SingleNavXGyro() {
        m_name = Names.name(this);

        // maximum update rate == minimum latency (use most-recent updates). maybe too
        // much CPU?
        m_gyro1 = new AHRS(SerialPort.Port.kUSB, SerialDataType.kProcessedData, kUpdateRateHz);
        m_gyro1.enableBoardlevelYawReset(true);

        Util.println("waiting for navx connection...");
        Timer.delay(2);

        while ((m_gyro1.isConnected() && m_gyro1.isCalibrating())) {
            Timer.delay(0.5);
            Util.println("Waiting for navx startup calibration...");
        }

        m_gyro1.zeroYaw();

        // periodic notifier so we can see it without any command running
        periodicLogger = new Notifier(this::logStuff);
        periodicLogger.setName("Gyro Periodic Logger Notifier");
        periodicLogger.startPeriodic(1);
    }

    /**
     * NOTE NOTE NOTE this is NED = clockwise positive = backwards
     * 
     * @returns yaw in degrees [-180,180]
     */
    @Override
    public float getYawNEDDeg() {
        float yawDeg = m_gyro1.getYaw();
        t.log(Level.TRACE, m_name, "Yaw NED (deg)", yawDeg);
        return yawDeg;
    }

    /**
     * @returns pitch in degrees [-180,180]
     */
    @Override
    public float getPitchDeg() {
        float pitchDeg = m_gyro1.getPitch();
        t.log(Level.TRACE, m_name, "Pitch (deg)", pitchDeg);
        return pitchDeg;
    }

    /**
     * @returns roll in degrees [-180,180]
     */
    @Override
    public float getRollDeg() {
        float rollDeg = m_gyro1.getRoll();
        t.log(Level.TRACE, m_name, "Roll (deg)", rollDeg);
        return rollDeg;
    }

    /**
     * NOTE NOTE NOTE this is NED = clockwise positive = backwards
     * 
     * @returns rate in degrees/sec
     */
    @Override
    public float getYawRateNEDDeg_s() {
        // 2/27/24 the NavX getRate() method has been broken since at least 2018
        //
        // https://github.com/kauailabs/navxmxp/issues/69
        //
        // the recommended workaround is to use getRawGyroZ() instead.
        float rateDeg_S = m_gyro1.getRawGyroZ();
        t.log(Level.TRACE, m_name, "Rate NED (rad_s)", rateDeg_S);
        return rateDeg_S;
    }

    private void logStuff() {
        if (m_gyro1.isConnected()) {
            t.log(Level.TRACE, m_name, "Connected", true);
        } else {
            t.log(Level.ERROR, m_name, "Connected", false);
        }
        t.log(Level.TRACE, m_name, "Angle (deg)", m_gyro1.getAngle());
        t.log(Level.TRACE, m_name, "Fused (deg)", m_gyro1.getFusedHeading());
        t.log(Level.TRACE, m_name, "Yaw (deg)", m_gyro1.getYaw());
        t.log(Level.TRACE, m_name, "Angle Mod 360 (deg)", m_gyro1.getAngle() % 360);
        t.log(Level.TRACE, m_name, "Compass Heading (deg)", m_gyro1.getCompassHeading());
    }
}
