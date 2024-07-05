package org.team100.lib.encoder.drive;

import java.util.OptionalDouble;

import org.team100.lib.encoder.SettableEncoder;
import org.team100.lib.motor.drive.Talon6DriveMotor;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;

public class Talon6DriveEncoder implements SettableEncoder<Distance100> {
    private final Logger m_logger;
    private final Talon6DriveMotor m_motor;
    private final double m_distancePerTurn;

    public Talon6DriveEncoder(
            Logger parent,
            Talon6DriveMotor motor,
            double distancePerTurn) {
        m_logger = parent.child(this);
        m_motor = motor;
        m_distancePerTurn = distancePerTurn;
    }

    /** Position in meters */
    @Override
    public OptionalDouble getPosition() {
        double motorPositionRev = m_motor.getPositionRev();
        double positionM = motorPositionRev * m_distancePerTurn;
        m_logger.logDouble(Level.TRACE,  "motor position (rev)", ()->motorPositionRev);
        m_logger.logDouble(Level.TRACE,  "position (m)",()-> positionM);
        return OptionalDouble.of(positionM);
    }

    /** Velocity in meters/sec */
    @Override
    public OptionalDouble getRate() {
        double motorVelocityRev_S = m_motor.getVelocityRev_S();
        double velocityM_S = motorVelocityRev_S * m_distancePerTurn;
        m_logger.logDouble(Level.TRACE,  "motor velocity (rev_s)", ()->motorVelocityRev_S);
        m_logger.logDouble(Level.TRACE,  "velocity (m_s)", ()->velocityM_S);
        return OptionalDouble.of(velocityM_S);
    }

    @Override
    public void setPosition(double positionM) {
        double motorPositionRev = positionM / m_distancePerTurn;
        m_motor.setEncoderPosition(motorPositionRev);
    }

    @Override
    public void reset() {
        m_motor.resetEncoderPosition();
    }

    @Override
    public void close() {
        m_motor.close();
    }
}
