package org.team100.lib.encoder.drive;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.drive.FalconDriveMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

/**
 * The built-in encoder in Falcon motors.
 * 
 * The encoder is a high-resolution magnetic sensor, 2048 ticks per turn.
 */
public class FalconDriveEncoder implements Encoder100<Distance100> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final FalconDriveMotor m_motor;
    private final double m_distancePerPulse;

    /** updated in periodic() */
    private double m_positionM;
    /** updated in periodic() */
    private double m_velocityM_S;

    /**
     * @param name            do not use a leading slash.
     * @param distancePerTurn in meters
     */
    public FalconDriveEncoder(
            String name,
            FalconDriveMotor motor,
            double distancePerTurn) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = Names.append(name, this);
        m_motor = motor;
        m_distancePerPulse = distancePerTurn / 2048;
    }

    /** Position in meters */
    @Override
    public double getPosition() {
        return m_positionM;
    }

    /** Velocity in meters/sec */
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

    ///////////////////////////////////

    private void updatePosition() {
        m_positionM = m_motor.getPosition() * m_distancePerPulse;
    }

    private void updateVelocity() {
        // sensor velocity is 1/2048ths of a turn per 100ms
        m_velocityM_S = m_motor.getVelocity2048_100() * 10 * m_distancePerPulse;
    }
}
