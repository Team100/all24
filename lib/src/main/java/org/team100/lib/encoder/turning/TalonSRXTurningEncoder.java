package org.team100.lib.encoder.turning;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.turning.CANTurningMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;

public class TalonSRXTurningEncoder implements Encoder100<Angle> {
    private static final int ticksPerRevolution = 1666;

    private final Telemetry t = Telemetry.get();
    private final WPI_TalonSRX m_motor;
    private final String m_name;

    public TalonSRXTurningEncoder(String name, CANTurningMotor motor) {
        m_motor = motor.getMotor();
        m_name = String.format("/TalonSRXEncoder %s", name);
    }

    @Override
    public double getPosition() {
        double rawPosition = m_motor.getSelectedSensorPosition();
        double angleRad = MathUtil.angleModulus(rawPosition / ticksPerRevolution * 2 * Math.PI);
        t.log(Level.DEBUG, m_name + "/Angle rad", angleRad);
        return angleRad;
    }

    @Override
    public double getRate() {
        // rate in ticks per 100ms.
        double rawRate = m_motor.getSelectedSensorVelocity();
        // times ten because rate is per 100ms
        double omegaRad_S = 2.0 * Math.PI * 10.0 * rawRate / ticksPerRevolution;
        t.log(Level.DEBUG, m_name + "/Omega rad_s", omegaRad_S);
        return omegaRad_S;
    }

    @Override
    public void reset() {
        m_motor.setSelectedSensorPosition(0);
    }

    @Override
    public void close() {
    }
}