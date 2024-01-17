package org.team100.lib.encoder.turning;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.turning.NeoTurningMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle;

/**
 * The built-in encoder in Neo motors.
 * 
 * This encoder simply senses the 14 rotor magnets in 3 places, so it's 42 ticks
 * per turn.
 */
public class NeoTurningEncoder implements Encoder100<Angle> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final NeoTurningMotor m_motor;

    /**
     * @param name            do not use a leading slash.
     * @param distancePerTurn in meters
     */
    public NeoTurningEncoder(
            String name,
            NeoTurningMotor motor) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = String.format("/%s/Neo Drive Encoder", name);
        m_motor = motor;
    }

    @Override
    public double getPosition() {
        // raw position is in rotations
        double result = m_motor.getPositionRot();
        t.log(Level.DEBUG, m_name + "/Position m", result);
        return result;
    }

    @Override
    public double getRate() {
        // raw velocity is in RPS
        double result = m_motor.getRateRPS();
        t.log(Level.DEBUG, m_name + "/Speed m_s", result);
        return result;

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
        //
    }
}
