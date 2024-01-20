package org.team100.lib.encoder.turning;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.turning.NeoTurningMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

/**
 * The built-in encoder in Neo motors.
 * 
 * This encoder simply senses the 14 rotor magnets in 3 places, so it's 42 ticks
 * per turn.
 */
public class NeoTurningEncoder implements Encoder100<Angle100> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final NeoTurningMotor m_motor;

    /** Current position measurement, obtained in periodic(). */
    private double m_positionRev;
    /** Current velocity measurement, obtained in periodic(). */
    private double m_rateRev_S;

    /**
     * @param name            do not use a leading slash.
     * @param distancePerTurn in meters
     */
    public NeoTurningEncoder(
            String name,
            NeoTurningMotor motor) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = Names.append(name, this);
        m_motor = motor;
    }

    /** Position in revolutions. */
    @Override
    public double getPosition() {
        return m_positionRev;
    }

    /** Velocity in revolutions per second. */
    @Override
    public double getRate() {
        return m_rateRev_S;
    }

    @Override
    public void reset() {
        m_motor.resetPosition();
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void periodic() {
        updatePosition();
        updateRate();
        t.log(Level.DEBUG, m_name, "position (rev)", m_positionRev);
        t.log(Level.DEBUG, m_name, "velocity (rev_s)", m_rateRev_S);
    }

    ////////////////////////////////////

    private void updatePosition() {
        m_positionRev = m_motor.getPositionRot();
    }

    private void updateRate() {
        m_rateRev_S = m_motor.getRateRPM() / 60;
    }
}
