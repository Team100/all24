package org.team100.lib.encoder.drive;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.drive.Talon6DriveMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

public class Talon6DriveEncoder implements Encoder100<Distance100> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final Talon6DriveMotor m_motor;
    private final double m_distancePerTurn;

    public Talon6DriveEncoder(
            String name,
            Talon6DriveMotor motor,
            double distancePerTurn) {
        m_name = Names.append(name, this);
        m_motor = motor;
        m_distancePerTurn = distancePerTurn;
    }

    /** Position in meters */
    @Override
    public Double getPosition() {
        double motorPositionRev = m_motor.getPositionRev();
        double positionM = motorPositionRev * m_distancePerTurn;
        t.log(Level.TRACE, m_name, "motor position (rev)", motorPositionRev);
        t.log(Level.DEBUG, m_name, "position (m)", positionM);
        return positionM;
    }

    /** Velocity in meters/sec */
    @Override
    public double getRate() {
        double motorVelocityRev_S = m_motor.getVelocityRev_S();
        double velocityM_S = motorVelocityRev_S * m_distancePerTurn;
        t.log(Level.TRACE, m_name, "motor velocity (rev_s)", motorVelocityRev_S);
        t.log(Level.DEBUG, m_name, "velocity (m_s)", velocityM_S);
        return velocityM_S;
    }

    @Override
    public void reset() {
        m_motor.resetPosition();
    }

    @Override
    public void close() {
        m_motor.close();
    }

}
