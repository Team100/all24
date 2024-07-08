package org.team100.lib.encoder.drive;

import java.util.OptionalDouble;

import org.team100.lib.encoder.SettableLinearEncoder;
import org.team100.lib.motor.NeoVortexCANSparkMotor;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

/**
 * The built-in encoder in Neo motors.
 * 
 * This encoder simply senses the 14 rotor magnets in 3 places, so it's 42 ticks
 * per turn.
 */
public class NeoVortexDriveEncoder implements SettableLinearEncoder {
    private final Logger m_logger;
    private final NeoVortexCANSparkMotor m_motor;
    private final double m_distancePerTurn;

    /**
     * @param distancePerTurn in meters
     */
    public NeoVortexDriveEncoder(
            Logger parent,
            NeoVortexCANSparkMotor motor,
            double distancePerTurn) {
        m_logger = parent.child(this);
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
        m_logger.logDouble(Level.TRACE, "position (m)", ()->positionM);
        return positionM;
    }

    private double getVelocityM_S() {
        // raw velocity is in RPM
        // this is fast, doesn't need to be cached
        double velocityM_S = m_motor.getRateRPM() * m_distancePerTurn / 60;
        m_logger.logDouble(Level.TRACE, "velocity (m_s)",()-> velocityM_S);
        return velocityM_S;
    }

}
