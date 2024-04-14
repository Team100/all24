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
    private final double m_gearRatio;

    /**
     * @param name            do not use a leading slash.
     * @param distancePerTurn in meters
     */
    public NeoTurningEncoder(
            String name,
            NeoTurningMotor motor,
            double gearRatio) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = Names.append(name, this);
        m_motor = motor;
        m_gearRatio = gearRatio;
        reset();
    }

    /** Position of the mechanism in radians. */
    @Override
    public Double getPosition() {
        return getPositionRad();
    }

    /** Velocity of the mechanism in radians per second. */
    @Override
    public double getRate() {
        return getRateRad_S();
    }

    @Override
    public void reset() {
        m_motor.resetPosition();
    }

    @Override
    public void close() {
        //
    }

    ////////////////////////////////////

    private double getPositionRad() {
        // should be fast, no need to cache it.
        double positionRad = m_motor.getPositionRot() * 2 * Math.PI / m_gearRatio;
        t.log(Level.DEBUG, m_name, "position (rad)", positionRad);
        return positionRad;
    }

    private double getRateRad_S() {
        // should be fast, no need to cache it.
        double rateRad_S = m_motor.getRateRPM() * 2 * Math.PI / (60 * m_gearRatio);
        t.log(Level.DEBUG, m_name, "velocity (rad_s)", rateRad_S);
        return rateRad_S;
    }
}
