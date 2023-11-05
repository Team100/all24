package org.team100.lib.encoder.turning;

import org.team100.lib.motor.turning.CANTurningMotor;
import org.team100.lib.telemetry.Telemetry;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;

public class TalonSRXTurningEncoder implements TurningEncoder {
    private static final int ticksPerRevolution = 1666;

    private final Telemetry t = Telemetry.get();
    private final WPI_TalonSRX m_motor;
    private final String m_name;

    public TalonSRXTurningEncoder(String name, CANTurningMotor motor) {
        m_motor = motor.getMotor();
        m_name = String.format("/TalonSRXEncoder %s", name);
    }

    @Override
    public double getAngle() {
        double angleRad = MathUtil.angleModulus(m_motor.getSelectedSensorPosition() / ticksPerRevolution * 2 * Math.PI);
        t.log(m_name + "/Angle rad", angleRad);
        return angleRad;
    }

    @Override
    public void reset() {
        m_motor.setSelectedSensorPosition(0);
    }

    @Override
    public void close() {
    }
}