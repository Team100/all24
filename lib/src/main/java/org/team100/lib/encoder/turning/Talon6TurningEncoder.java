package org.team100.lib.encoder.turning;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.turning.Talon6TurningMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;

public class Talon6TurningEncoder implements Encoder100<Angle100> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final Talon6TurningMotor m_motor;
    private final double m_gearRatio;

    public Talon6TurningEncoder(
            String m_name,
            Talon6TurningMotor m_motor,
            double m_gearRatio) {
        this.m_name = m_name;
        this.m_motor = m_motor;
        this.m_gearRatio = m_gearRatio;
    }

    /** Position in radians */
    @Override
    public Double getPosition() {
        double motorPositionRev = m_motor.getPositionRev();
        double positionRad = motorPositionRev * 2 * Math.PI / m_gearRatio;
        t.log(Level.TRACE, m_name, "motor position (rev)", motorPositionRev);
        t.log(Level.DEBUG, m_name, "output position (rad)", positionRad);
        return positionRad;
    }

    /** Velocity in rad/s */
    @Override
    public double getRate() {
        double motorVelocityRev_S = m_motor.getVelocityRev_S();
        double outputVelocityRad_S = motorVelocityRev_S * 2 * Math.PI / m_gearRatio;
        t.log(Level.TRACE, m_name, "motor velocity (rev_s)", motorVelocityRev_S);
        t.log(Level.DEBUG, m_name, "output velocity (rad_s)", outputVelocityRad_S);
        return outputVelocityRad_S;
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
