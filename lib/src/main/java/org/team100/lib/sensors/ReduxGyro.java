package org.team100.lib.sensors;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class ReduxGyro implements Gyro {

    private final Canandgyro m_gyro;

    // LOGGERS
    private final DoubleLogger m_log_heading;
    private final DoubleLogger m_log_heading_rate;
    private final DoubleLogger m_log_pitch;
    private final DoubleLogger m_log_roll;
    private final DoubleLogger m_log_yaw_deg;
    private final DoubleLogger m_log_pitch_deg;
    private final DoubleLogger m_log_roll_deg;
    public ReduxGyro(LoggerFactory parent) {
        LoggerFactory child = parent.child(this);
        m_gyro = new Canandgyro(60);
        // m_gyro.setYaw(0);
        m_log_heading = child.doubleLogger(Level.TRACE, "Heading NWU (rad)");
        m_log_heading_rate = child.doubleLogger(Level.TRACE, "Heading Rate NWU (rad_s)");
        m_log_pitch = child.doubleLogger(Level.TRACE, "Pitch NWU (rad)");
        m_log_roll = child.doubleLogger(Level.TRACE, "Roll NWU (rad)");
        m_log_yaw_deg = child.doubleLogger(Level.DEBUG, "Yaw NED (deg)");
        m_log_pitch_deg = child.doubleLogger(Level.TRACE, "Pitch (deg)");
        m_log_roll_deg = child.doubleLogger(Level.TRACE, "Roll (deg)");

    }

    @Override
    public Rotation2d getYawNWU() {
        Rotation2d currentHeadingNWU = Rotation2d.fromDegrees(-1.0 * getYawNEDDeg());
        m_log_heading.log(currentHeadingNWU::getRadians);
        return currentHeadingNWU;
    }

    @Override
    public double getYawRateNWU() {
        double m_yawRateRad_S = -360 * m_gyro.getAngularVelocityYaw();
        m_log_heading_rate.log(() -> m_yawRateRad_S);
        return m_yawRateRad_S;
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

    private double getYawNEDDeg() {
        double yawDeg = 360 * m_gyro.getYaw();
        m_log_yaw_deg.log(() -> yawDeg);
        return yawDeg;
    }

    private double getPitchDeg() {
        double pitchDeg = 360 * m_gyro.getPitch();
        m_log_pitch_deg.log(() -> pitchDeg);
        return pitchDeg;
    }

    /**
     * @returns roll in degrees [-180,180]
     */
    private double getRollDeg() {
        double rollDeg = 360 * m_gyro.getRoll();
        m_log_roll_deg.log(() -> rollDeg);
        return rollDeg;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }
}
