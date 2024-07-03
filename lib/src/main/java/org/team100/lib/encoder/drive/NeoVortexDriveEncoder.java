package org.team100.lib.encoder.drive;

import java.util.OptionalDouble;

import org.team100.lib.encoder.SettableEncoder;
import org.team100.lib.motor.drive.NeoVortexDriveMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

/**
 * The built-in encoder in Neo motors.
 * 
 * This encoder simply senses the 14 rotor magnets in 3 places, so it's 42 ticks
 * per turn.
 */
public class NeoVortexDriveEncoder implements SettableEncoder<Distance100> {
    private final Telemetry.Logger t;
    private final String m_name;
    private final NeoVortexDriveMotor m_motor;
    private final double m_distancePerTurn;

    /**
     * @param name            do not use a leading slash.
     * @param distancePerTurn in meters
     */
    public NeoVortexDriveEncoder(
            String name,
            NeoVortexDriveMotor motor,
            double distancePerTurn) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = Names.append(name, this);
        t = Telemetry.get().logger(m_name);
        m_motor = motor;
        m_distancePerTurn = distancePerTurn;
    }

    /** Position in meters. */
    @Override
    public OptionalDouble getPosition() {
        return OptionalDouble.of(getPositionM());
    }

    /** Velocity in meters/sec. */
    @Override
    public OptionalDouble getRate() {
        return OptionalDouble.of(getVelocityM_S());
    }

    /** Position in meters. */
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
        //
    }

    //////////////////////////////////

    private double getPositionM() {
        // raw position is in rotations
        // this is fast, doesn't need to be cached
        double positionM = m_motor.getPositionRot() * m_distancePerTurn;
        t.logDouble(Level.DEBUG, "position (m)", ()->positionM);
        return positionM;
    }

    private double getVelocityM_S() {
        // raw velocity is in RPM
        // this is fast, doesn't need to be cached
        double velocityM_S = m_motor.getRateRPM() * m_distancePerTurn / 60;
        t.logDouble(Level.DEBUG, "velocity (m_s)",()-> velocityM_S);
        return velocityM_S;
    }

}
