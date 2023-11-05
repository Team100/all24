package org.team100.lib.sensors;

import org.team100.lib.config.Identity;
import org.team100.lib.telemetry.Telemetry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

// TODO: figure out why we disabled this mechanism during the 2023 season
public class RedundantGyro implements RedundantGyroInterface {

    /** For robots without a gyro. */
    private static class Noop implements RedundantGyroInterface {

        @Override
        public float getRedundantGyroRateNED() {
            return 0;
        }

        @Override
        public float getRedundantGyroZ() {
            return 0;
        }

        @Override
        public float getRedundantPitch() {
            return 0;
        }

        @Override
        public float getRedundantRoll() {
            return 0;
        }

        @Override
        public float getRedundantYawNED() {
            return 0;
        }
    }

    public static class Factory {
        private final Identity m_identity;

        public Factory(Identity identity) {
            m_identity = identity;
        }

        public RedundantGyroInterface get() {
            switch (m_identity) {
                case COMP_BOT:
                case SWERVE_ONE:
                case SWERVE_TWO:
                    return new RedundantGyro();
                default:
                    return new Noop();
            }
        }
    }

    private final Telemetry t = Telemetry.get();

    private final AHRS m_gyro1;
    private final AHRS m_gyro2;
    private final Timer m_timer;

    private float gyroZOffset_I2C;
    private float gyroZOffset_USB;
    private boolean timeGap;
    private final Notifier periodicLogger;

    public RedundantGyro() {

        timeGap = false;

        m_timer = new Timer();
        m_timer.start();

        m_gyro1 = new AHRS(SerialPort.Port.kUSB);
        m_gyro2 = new AHRS(I2C.Port.kMXP);
        m_gyro1.enableBoardlevelYawReset(true);
        m_gyro2.enableBoardlevelYawReset(true);
        m_gyro1.calibrate();
        m_gyro2.calibrate();

        while (m_timer.get() < 2) {
            // wait a bit
        }

        while ((m_gyro1.isConnected() && m_gyro1.isCalibrating() || m_gyro2.isConnected() && m_gyro2.isCalibrating())
                || timeGap) {
            // wait
        }

        m_gyro1.zeroYaw();
        m_gyro2.zeroYaw();

        gyroZOffset_I2C = -m_gyro2.getRawGyroZ();
        gyroZOffset_USB = -m_gyro1.getRawGyroZ();
        periodicLogger = new Notifier(this::logStuff);
        periodicLogger.startPeriodic(1);
    }

    /** NOTE NOTE NOTE this is NED = clockwise positive = backwards */
    public float getRedundantYawNED() {
        float redundYaw = 0;
        int tmpInputs = 0;
        if (m_gyro1.isConnected()) {
            connected1(true);
            redundYaw += m_gyro1.getYaw();
            tmpInputs += 1;
        } else {
            connected1(false);
        }
        if (m_gyro2.isConnected()) {
            connected2(true);
            redundYaw += m_gyro2.getYaw();
            tmpInputs += 1;
        } else {
            connected2(false);
        }
        totalConnected(tmpInputs);

        // this just returns the first one.
        // TODO fix this
        float result = m_gyro1.getYaw();

        t.log("/RedundantGyro/Gyro Redundant Yaw NED (rad)", result);

        return result;
    }

    /**
     * TODO: is this really degrees?
     * 
     * @returns pitch in degrees [-180,180]
     */
    public float getRedundantPitch() {
        float redundPitch = 0;
        int tmpInputs = 0;
        if (m_gyro1.isConnected()) {
            connected1(true);
            redundPitch += m_gyro1.getPitch();
            tmpInputs += 1;
        } else {
            connected1(false);
        }

        if (m_gyro2.isConnected()) {
            connected2(true);
            redundPitch += m_gyro2.getPitch();
            tmpInputs += 1;
        } else {
            connected2(false);
        }

        totalConnected(tmpInputs);

        float result = redundPitch / tmpInputs;

        t.log("/RedundantGyro/Gyro Redundant Pitch (deg)", result);

        return result;
    }

    /**
     * TODO: is this really degrees?
     * 
     * @returns roll in degrees [-180,180]
     */
    public float getRedundantRoll() {
        float redundRoll = 0;
        int tmpInputs = 0;
        if (m_gyro1.isConnected()) {
            connected1(true);
            redundRoll += m_gyro1.getRoll();
            tmpInputs += 1;
        } else {
            connected1(false);
        }
        if (m_gyro2.isConnected()) {
            connected2(true);
            redundRoll += m_gyro2.getRoll();
            tmpInputs += 1;
        } else {
            connected2(false);
        }

        totalConnected(tmpInputs);

        float result = redundRoll / tmpInputs;

        t.log("/RedundantGyro/Gyro Redundant Roll (deg)", result);

        return result;
    }

    /**
     * NOTE this is NED = clockwise positive = backwards
     * TODO: check the units
     */
    public float getRedundantGyroRateNED() {
        float redundRate = 0;
        int tmpInputs = 0;
        if (m_gyro1.isConnected()) {
            connected1(true);
            redundRate += m_gyro1.getRate();
            tmpInputs += 1;
        } else {
            connected1(false);
        }
        if (m_gyro2.isConnected()) {
            connected2(true);
            redundRate += m_gyro2.getRate();
            tmpInputs += 1;
        } else {
            connected2(false);
        }

        totalConnected(tmpInputs);

        float result = (redundRate) / tmpInputs;

        t.log("/RedundantGyro/Gyro Redundant Rate NED (rad/s)", result);

        return result;
    }

    // do we need this ?
    public float getRedundantGyroZ() {
        float redundGyroZ = 0;
        int tmpInputs = 0;
        if (m_gyro1.isConnected()) {
            connected1(true);
            redundGyroZ += m_gyro1.getRawGyroZ() + gyroZOffset_USB;
            tmpInputs += 1;
        } else {
            connected1(false);
        }
        if (m_gyro2.isConnected()) {
            connected2(true);
            redundGyroZ += m_gyro2.getRawGyroZ() + gyroZOffset_I2C;
            tmpInputs += 1;
        } else {
            connected2(false);
        }

        totalConnected(tmpInputs);
        float result = redundGyroZ / tmpInputs;

        t.log("/RedundantGyro/Gyro Z ", result);

        return result;
    }

    private void connected1(boolean connected) {
        t.log("/RedundantGyro/Gyro 1 Connected", connected);
    }

    private void connected2(boolean connected) {
        t.log("/RedundantGyro/Gyro 2 Connected", connected);
    }

    private void totalConnected(int connected) {
        t.log("/RedundantGyro/Total Connected", connected);
    }

    public void logStuff() {
        t.log("/RedundantGyro/Gyro 1 Angle (deg)", m_gyro1.getAngle());
        t.log("/RedundantGyro/Gyro 2 Angle (deg)", m_gyro2.getAngle());
        t.log("/RedundantGyro/Gyro 1 Fused (deg)", m_gyro1.getFusedHeading());
        t.log("/RedundantGyro/Gyro 2 Fused (deg)", m_gyro2.getFusedHeading());
        t.log("/RedundantGyro/Gyro 1 Yaw", m_gyro1.getYaw());
        t.log("/RedundantGyro/Gyro 2 Yaw", m_gyro2.getYaw());
        t.log("/RedundantGyro/Gyro 1 Angle Mod 360 (deg)", m_gyro1.getAngle() % 360);
        t.log("/RedundantGyro/Gyro 2 Angle Mod 360 (deg)", m_gyro2.getAngle() % 360);
        t.log("/RedundantGyro/Gyro 1 Compass Heading (deg)", m_gyro1.getCompassHeading());
        t.log("/RedundantGyro/Gyro 2 Compass Heading (deg)", m_gyro2.getCompassHeading());

    }
}
