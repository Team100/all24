package org.team100.lib.encoder.drive;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.drive.FalconDriveMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance;

public class FalconDriveEncoder implements Encoder100<Distance> {
    private final Telemetry t = Telemetry.get();

    private final FalconDriveMotor m_motor;
    private final double m_distancePerPulse;
    private final String m_name;

    /** @param distancePerTurn in meters */
    public FalconDriveEncoder(String name,
            FalconDriveMotor motor,
            double distancePerTurn) {
        this.m_motor = motor;
        this.m_distancePerPulse = distancePerTurn / 2048;
        m_name = String.format("/%s/Falcon Drive Encoder", name);
    }

    @Override
    public double getPosition() {
        return m_motor.getPosition() * m_distancePerPulse;
    }

    @Override
    public double getRate() {
        // sensor velocity is 1/2048ths of a turn per 100ms
        double result = m_motor.getVelocity2048_100() * 10 * m_distancePerPulse;
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
}
