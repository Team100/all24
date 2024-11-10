package org.team100.lib.sensors;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.util.Util;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.Canandgyro.Faults;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ReduxGyro implements Gyro {

    private final Canandgyro m_gyro;

    // LOGGERS
    private final Rotation2dLogger m_log_yaw;
    private final DoubleLogger m_log_yaw_rate;
    private final Rotation2dLogger m_log_pitch;
    private final Rotation2dLogger m_log_roll;

    public ReduxGyro(LoggerFactory parent, int canID) {
        LoggerFactory child = parent.child(this);
        m_gyro = new Canandgyro(canID);
        m_gyro.clearStickyFaults();
        m_gyro.setYaw(0);
        m_log_yaw = child.rotation2dLogger(Level.TRACE, "Yaw NWU (rad)");
        m_log_yaw_rate = child.doubleLogger(Level.TRACE, "Yaw Rate NWU (rad_s)");
        m_log_pitch = child.rotation2dLogger(Level.TRACE, "Pitch NWU (rad)");
        m_log_roll = child.rotation2dLogger(Level.TRACE, "Roll NWU (rad)");

    }

    @Override
    public Rotation2d getYawNWU() {
        Rotation2d yawNWU = Rotation2d.fromRotations(m_gyro.getYaw());
        m_log_yaw.log(() -> yawNWU);
        return yawNWU;
    }

    @Override
    public double getYawRateNWU() {
        double yawRateRad_S = Units.rotationsToRadians(m_gyro.getAngularVelocityYaw());
        m_log_yaw_rate.log(() -> yawRateRad_S);
        return yawRateRad_S;
    }

    @Override
    public Rotation2d getPitchNWU() {
        Rotation2d pitchNWU = Rotation2d.fromRotations(m_gyro.getPitch());
        m_log_pitch.log(() -> pitchNWU);
        return pitchNWU;
    }

    @Override
    public Rotation2d getRollNWU() {
        Rotation2d rollNWU = Rotation2d.fromRotations(m_gyro.getRoll());
        m_log_roll.log(() -> rollNWU);
        return rollNWU;
    }

    @Override
    public void periodic() {
        if (m_gyro.isCalibrating())
            Util.println("Redux Gyro Calibrating ......");
        Faults activeFaults = m_gyro.getActiveFaults();
        if (activeFaults.faultsValid())
            Util.warn("Redux Gyro fault!");

    }
}