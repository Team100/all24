package org.team100.lib.sensors;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

/**
 * Combine two NavX AHRS to increase reliability.
 */
public class RedundantGyro implements RedundantGyroInterface {
    private final Telemetry t = Telemetry.get();
    private final AHRS m_gyro1;
    private final AHRS m_gyro2;
    private final Notifier periodicLogger;

    public RedundantGyro() {

        m_gyro1 = new AHRS(SerialPort.Port.kUSB);
        m_gyro1.enableBoardlevelYawReset(true);

        m_gyro2 = new AHRS(I2C.Port.kMXP);
        m_gyro2.enableBoardlevelYawReset(true);

        Util.println("waiting for navx connection...");
        Timer.delay(2);

        while ((m_gyro1.isConnected() && m_gyro1.isCalibrating())
                || (m_gyro2.isConnected() && m_gyro2.isCalibrating())) {
            Timer.delay(0.5);
            Util.println("Waiting for navx startup calibration...");
        }

        m_gyro1.zeroYaw();
        m_gyro2.zeroYaw();

        // periodic notifier so we can see it without any command running
        periodicLogger = new Notifier(this::logStuff);
        periodicLogger.startPeriodic(1);
    }

    /**
     * NOTE NOTE NOTE this is NED = clockwise positive = backwards
     * 
     * @returns yaw in degrees [-180,180]
     */
    public float getRedundantYawNED() {
        float yawDeg = 0;
        int inputs = 0;
        if (m_gyro1.isConnected()) {
            connected1(true);
            yawDeg += m_gyro1.getYaw();
            inputs += 1;
        } else {
            connected1(false);
        }
        if (m_gyro2.isConnected()) {
            connected2(true);
            yawDeg += m_gyro2.getYaw();
            inputs += 1;
        } else {
            connected2(false);
        }
        totalConnected(inputs);

        float result = inputs == 0 ? 0 : yawDeg / inputs;

        t.log(Level.DEBUG, "/RedundantGyro/Gyro Redundant Yaw NED (rad)", result);

        return result;
    }

    /**
     * @returns pitch in degrees [-180,180]
     */
    public float getRedundantPitch() {
        float pitchDeg = 0;
        int inputs = 0;
        if (m_gyro1.isConnected()) {
            connected1(true);
            pitchDeg += m_gyro1.getPitch();
            inputs += 1;
        } else {
            connected1(false);
        }

        if (m_gyro2.isConnected()) {
            connected2(true);
            pitchDeg += m_gyro2.getPitch();
            inputs += 1;
        } else {
            connected2(false);
        }

        totalConnected(inputs);

        float result = inputs == 0 ? 0 : pitchDeg / inputs;

        t.log(Level.DEBUG, "/RedundantGyro/Gyro Redundant Pitch (deg)", result);

        return result;
    }

    /**
     * @returns roll in degrees [-180,180]
     */
    public float getRedundantRoll() {
        float rollDeg = 0;
        int inputs = 0;
        if (m_gyro1.isConnected()) {
            connected1(true);
            rollDeg += m_gyro1.getRoll();
            inputs += 1;
        } else {
            connected1(false);
        }
        if (m_gyro2.isConnected()) {
            connected2(true);
            rollDeg += m_gyro2.getRoll();
            inputs += 1;
        } else {
            connected2(false);
        }

        totalConnected(inputs);

        float result = inputs == 0 ? 0 : rollDeg / inputs;

        t.log(Level.DEBUG, "/RedundantGyro/Gyro Redundant Roll (deg)", result);

        return result;
    }

    /**
     * NOTE NOTE NOTE this is NED = clockwise positive = backwards
     * 
     * @returns rate in deg/sec
     */
    public float getRedundantGyroRateNED() {
        float rateDeg_S = 0;
        int inputs = 0;
        if (m_gyro1.isConnected()) {
            connected1(true);
            rateDeg_S += m_gyro1.getRate();
            inputs += 1;
        } else {
            connected1(false);
        }
        if (m_gyro2.isConnected()) {
            connected2(true);
            rateDeg_S += m_gyro2.getRate();
            inputs += 1;
        } else {
            connected2(false);
        }

        totalConnected(inputs);

        float result = inputs == 0 ? 0 : rateDeg_S / inputs;

        t.log(Level.DEBUG, "/RedundantGyro/Gyro Redundant Rate NED (rad/s)", result);

        return result;
    }

    private void connected1(boolean connected) {
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 1 Connected", connected);
    }

    private void connected2(boolean connected) {
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 2 Connected", connected);
    }

    private void totalConnected(int connected) {
        t.log(Level.DEBUG, "/RedundantGyro/Total Connected", connected);
    }

    private void logStuff() {
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 1 Angle (deg)", m_gyro1.getAngle());
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 2 Angle (deg)", m_gyro2.getAngle());
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 1 Fused (deg)", m_gyro1.getFusedHeading());
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 2 Fused (deg)", m_gyro2.getFusedHeading());
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 1 Yaw", m_gyro1.getYaw());
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 2 Yaw", m_gyro2.getYaw());
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 1 Angle Mod 360 (deg)", m_gyro1.getAngle() % 360);
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 2 Angle Mod 360 (deg)", m_gyro2.getAngle() % 360);
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 1 Compass Heading (deg)", m_gyro1.getCompassHeading());
        t.log(Level.DEBUG, "/RedundantGyro/Gyro 2 Compass Heading (deg)", m_gyro2.getCompassHeading());
    }
}
