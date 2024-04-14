package org.team100.lib.encoder.turning;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.turning.CANTurningMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;

/**
 * This was an attempt to use the TalonSRX data interface for absolute encoder,
 * but it didn't work out. Maybe we should delete it.
 */
public class TalonSRXTurningEncoder implements Encoder100<Angle100> {
    private static final int ticksPerRevolution = 1666;

    private final Telemetry t = Telemetry.get();
    private final WPI_TalonSRX m_motor;
    private final String m_name;

    public TalonSRXTurningEncoder(String name, CANTurningMotor motor) {
        m_motor = motor.getMotor();
        m_name = Names.append(name, this);
    }

    @Override
    public Double getPosition() {
        return getPositionRad();
    }

    @Override
    public double getRate() {
        return getRateRad_S();
    }

    @Override
    public void reset() {
        m_motor.setSelectedSensorPosition(0);
    }

    @Override
    public void close() {
        //
    }

    //////////////////////////////////////////////

    private double getPositionRad() {
        // should be fast, no need to cache
        double rawPosition = m_motor.getSelectedSensorPosition();
        double positionRad = MathUtil.angleModulus(rawPosition / ticksPerRevolution * 2 * Math.PI);
        t.log(Level.DEBUG, m_name, "position (rad)", positionRad);
        return positionRad;
    }

    private double getRateRad_S() {
        // rate in ticks per 100ms.
        // should be fast, no need to cache
        double rawRate = m_motor.getSelectedSensorVelocity();
        // times ten because rate is per 100ms
        double rateRad_S = 2.0 * Math.PI * 10.0 * rawRate / ticksPerRevolution;
        t.log(Level.DEBUG, m_name, "velocity (rad_s)", rateRad_S);
        return rateRad_S;
    }

}