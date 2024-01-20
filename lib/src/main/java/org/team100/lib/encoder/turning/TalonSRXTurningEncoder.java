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

    /** Current position, updated in periodic(); */
    private double m_positionRad;
    /** Current velocity, updated in periodic(); */
    private double m_rateRadS;

    public TalonSRXTurningEncoder(String name, CANTurningMotor motor) {
        m_motor = motor.getMotor();
        m_name = Names.append(name, this);
    }

    @Override
    public double getPosition() {
        return m_positionRad;
    }

    @Override
    public double getRate() {
        return m_rateRadS;
    }

    @Override
    public void reset() {
        m_motor.setSelectedSensorPosition(0);
        m_positionRad = 0;
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void periodic() {
        updatePosition();
        updateRate();
        t.log(Level.DEBUG, m_name, "position (rad)", m_positionRad);
        t.log(Level.DEBUG, m_name, "velocity (rad_s)", m_rateRadS);
    }

    //////////////////////////////////////////////

    private void updatePosition() {
        double rawPosition = m_motor.getSelectedSensorPosition();
        m_positionRad = MathUtil.angleModulus(rawPosition / ticksPerRevolution * 2 * Math.PI);
    }

    private void updateRate() {
        // rate in ticks per 100ms.
        double rawRate = m_motor.getSelectedSensorVelocity();
        // times ten because rate is per 100ms
        m_rateRadS = 2.0 * Math.PI * 10.0 * rawRate / ticksPerRevolution;
    }

}