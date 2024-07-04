package org.team100.lib.encoder.turning;

import java.util.OptionalDouble;

import org.team100.lib.encoder.SettableEncoder;
import org.team100.lib.motor.turning.NeoVortexTurningMotor;
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
public class NeoVortexTurningEncoder implements SettableEncoder<Angle100> {
    private final Logger m_logger;
    private final String m_name;
    private final NeoVortexTurningMotor m_motor;
    private final double m_gearRatio;

    /**
     * @param name            do not use a leading slash.
     * @param distancePerTurn in meters
     */
    public NeoVortexTurningEncoder(
            String name,
            Logger parent,
            NeoVortexTurningMotor motor,
            double gearRatio) {
        m_name = Names.append(name, this);
        m_logger = parent.child(this);
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
        // should be fast, no need to cache
        double positionRad = m_motor.getPositionRot() * 2 * Math.PI / m_gearRatio;
        m_logger.logDouble(Level.DEBUG,  "position (rad)", ()->positionRad);
        return positionRad;
    }

    private double getRateRad_S() {
        // should be fast, no need to cache
        double rateRad_S = m_motor.getRateRPM() * 2 * Math.PI / (60 * m_gearRatio);
        m_logger.logDouble(Level.DEBUG,  "velocity (rad_s)", ()->rateRad_S);
        return rateRad_S;
    }
}
