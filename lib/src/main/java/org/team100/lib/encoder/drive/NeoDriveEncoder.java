package org.team100.lib.encoder.drive;

import java.util.OptionalDouble;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.drive.NeoDriveMotor;
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
public class NeoDriveEncoder implements Encoder100<Distance100> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final NeoDriveMotor m_motor;
    private final double m_distancePerTurn;

    /**
     * @param name            do not use a leading slash.
     * @param distancePerTurn in meters
     */
    public NeoDriveEncoder(
            String name,
            NeoDriveMotor motor,
            double distancePerTurn) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = Names.append(name, this);
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

    @Override
    public void reset() {
        m_motor.resetPosition();
    }

    @Override
    public void close() {
        //
    }

    //////////////////////////////////

    private double getPositionM() {
        // raw position is in rotations
        // this is fast so we don't need to cache it
        double motorPositionRev = m_motor.getPositionRot();
        double positionM = motorPositionRev * m_distancePerTurn;
        t.log(Level.TRACE, m_name, "motor position (rev)", motorPositionRev);
        t.log(Level.DEBUG, m_name, "position (m)", positionM);
        return positionM;
    }

    private double getVelocityM_S() {
        // raw velocity is in RPM
        // this is fast so we don't need to cache it
        double velocityM_S = m_motor.getRateRPM() * m_distancePerTurn / 60;
        t.log(Level.DEBUG, m_name, "velocity (m_s)", velocityM_S);
        return velocityM_S;
    }

}
