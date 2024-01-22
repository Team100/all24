package org.team100.lib.encoder.drive;

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

    /** updated in periodic() */
    private double m_positionM;
    /** updated in periodic() */
    private double m_velocityM_S;

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
    public double getPosition() {
        return m_positionM;
    }

    /** Velocity in meters/sec. */
    @Override
    public double getRate() {
        return m_velocityM_S;
    }

    @Override
    public void reset() {
        m_motor.resetPosition();
        m_positionM = 0;
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void periodic() {
        updatePosition();
        updateVelocity();
        t.log(Level.DEBUG, m_name, "position (m)", m_positionM);
        t.log(Level.DEBUG, m_name, "velocity (m_s)", m_velocityM_S);
    }

    //////////////////////////////////

    private void updatePosition() {
        // raw position is in rotations
        m_positionM = m_motor.getPositionRot() * m_distancePerTurn;
    }

    private void updateVelocity() {
        // raw velocity is in MPS
        m_velocityM_S = m_motor.getRateRPM() * m_distancePerTurn / 60;
    }

}
