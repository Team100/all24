package org.team100.lib.encoder.drive;

import java.util.OptionalDouble;

import org.team100.lib.encoder.SettableEncoder;
import org.team100.lib.motor.drive.Talon6DriveMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

public class Talon6DriveEncoder implements SettableEncoder<Distance100> {
    private final Telemetry.Logger t;
    private final String m_name;
    private final Talon6DriveMotor m_motor;
    private final double m_distancePerTurn;

    public Talon6DriveEncoder(
            String name,
            Talon6DriveMotor motor,
            double distancePerTurn) {
        m_name = Names.append(name, this);
        t = Telemetry.get().logger(m_name);
        m_motor = motor;
        m_distancePerTurn = distancePerTurn;
    }

    /** Position in meters */
    @Override
    public OptionalDouble getPosition() {
        double motorPositionRev = m_motor.getPositionRev();
        double positionM = motorPositionRev * m_distancePerTurn;
        t.log(Level.TRACE,  "motor position (rev)", motorPositionRev);
        t.log(Level.DEBUG,  "position (m)", positionM);
        return OptionalDouble.of(positionM);
    }

    /** Velocity in meters/sec */
    @Override
    public OptionalDouble getRate() {
        double motorVelocityRev_S = m_motor.getVelocityRev_S();
        double velocityM_S = motorVelocityRev_S * m_distancePerTurn;
        t.log(Level.TRACE,  "motor velocity (rev_s)", motorVelocityRev_S);
        t.log(Level.DEBUG,  "velocity (m_s)", velocityM_S);
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
