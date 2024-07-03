package org.team100.lib.encoder.turning;

import java.util.OptionalDouble;

import org.team100.lib.encoder.SettableEncoder;
import org.team100.lib.motor.turning.NeoTurningMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

/**
 * The built-in encoder in Neo motors.
 * 
 * This encoder simply senses the 14 rotor magnets in 3 places, so it's 42 ticks
 * per turn.
 */
public class NeoTurningEncoder implements SettableEncoder<Angle100> {
    private final Telemetry.Logger t;
    private final String m_name;
    private final NeoTurningMotor m_motor;
    private final double m_gearRatio;

    /**
     * @param name            do not use a leading slash.
     * @param distancePerTurn in meters
     */
    public NeoTurningEncoder(
            String name,
            Logger parent,
            NeoTurningMotor motor,
            double gearRatio) {
        m_name = Names.append(name, this);
        t = Telemetry.get().logger(m_name, parent);
        m_motor = motor;
        m_gearRatio = gearRatio;
        reset();
    }

    /** Position of the mechanism in radians. */
    @Override
    public OptionalDouble getPosition() {
        return OptionalDouble.of(getPositionRad());
    }

    /** Velocity of the mechanism in radians per second. */
    @Override
    public OptionalDouble getRate() {
        return OptionalDouble.of(getRateRad_S());
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
        //
    }

    ////////////////////////////////////

    private double getPositionRad() {
        // should be fast, no need to cache it.
        double motorPositionRev = m_motor.getPositionRot();
        double positionRad = motorPositionRev * 2 * Math.PI / m_gearRatio;
        t.logDouble(Level.DEBUG,  "motor position (rev)",()-> motorPositionRev);
        t.logDouble(Level.DEBUG,  "output position (rad)",()-> positionRad);
        return positionRad;
    }

    private double getRateRad_S() {
        // should be fast, no need to cache it.
        double motorVelocityRev_S = m_motor.getRateRPM() / 60;
        double outputVelocityRad_S = motorVelocityRev_S * 2 * Math.PI / m_gearRatio;
        t.logDouble(Level.DEBUG,  "motor velocity (rev_s)", ()->motorVelocityRev_S);
        t.logDouble(Level.DEBUG,  "output velocity (rad_s)", ()->outputVelocityRad_S);
        return outputVelocityRad_S;
    }
}
