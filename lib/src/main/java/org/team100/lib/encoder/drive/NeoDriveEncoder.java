package org.team100.lib.encoder.drive;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.drive.NeoDriveMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance;

/**
 * The built-in encoder in Neo motors.
 * 
 * This encoder simply senses the 14 rotor magnets in 3 places, so it's 42 ticks
 * per turn.
 */
public class NeoDriveEncoder implements Encoder100<Distance> {
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
        m_name = String.format("/%s/Neo Drive Encoder", name);
        m_motor = motor;
        m_distancePerTurn = distancePerTurn;
    }

    @Override
    public double getPosition() {
        // raw position is in rotations
        double result = m_motor.getPositionRot() * m_distancePerTurn;
        t.log(Level.DEBUG, m_name + "/Position m", result);
        return result;
    }

    @Override
    public double getRate() {
        // raw velocity is in RPM
        double result = m_motor.getRateRPM() * m_distancePerTurn / 60;
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
