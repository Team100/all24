package org.team100.lib.encoder.turning;

import java.util.OptionalDouble;

import org.team100.lib.encoder.SettableAngularEncoder;
import org.team100.lib.motor.Talon6Motor;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

public class Talon6TurningEncoder implements SettableAngularEncoder {
    private final Logger m_logger;
    private final Talon6Motor m_motor;
    private final double m_gearRatio;

    public Talon6TurningEncoder(
            Logger parent,
            Talon6Motor motor,
            double gearRatio) {
        m_logger = parent.child(this);
        m_motor = motor;
        m_gearRatio = gearRatio;
    }

    /** Position in radians */
    @Override
    public OptionalDouble getPosition() {
        double motorPositionRev = m_motor.getPositionRev();
        double positionRad = motorPositionRev * 2 * Math.PI / m_gearRatio;
        m_logger.logDouble(Level.TRACE,  "motor position (rev)", ()->motorPositionRev);
        m_logger.logDouble(Level.TRACE,  "output position (rad)",()-> positionRad);
        return OptionalDouble.of(positionRad);
    }

    /** Velocity in rad/s */
    @Override
    public OptionalDouble getRate() {
        double motorVelocityRev_S = m_motor.getVelocityRev_S();
        double outputVelocityRad_S = motorVelocityRev_S * 2 * Math.PI / m_gearRatio;
        m_logger.logDouble(Level.TRACE,  "motor velocity (rev_s)",()-> motorVelocityRev_S);
        m_logger.logDouble(Level.TRACE,  "output velocity (rad_s)",()->outputVelocityRad_S);
        return OptionalDouble.of(outputVelocityRad_S);
    }

    @Override
    public void setPosition(double positionRad) {
        double motorPositionRev = positionRad * m_gearRatio / (2 * Math.PI);
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
