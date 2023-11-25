package org.team100.lib.motion.simple;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a simple one-dimensional mechanism. */
public class SimpleSubsystem extends SubsystemBase {
    public static class Config {
        public double filterTimeConstantS = 0.06;
        public double filterPeriodS = 0.02;
        public double encoderZero = 0.861614;
    }

    private final Config m_config = new Config();
    private final Telemetry t = Telemetry.get();
    private final LinearFilter m_filter;
    private final CANSparkMax m_motor;
    private final AnalogEncoder m_encoder;
    private double m_previousPosition;

    public SimpleSubsystem() {
        m_filter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);

        m_motor = new CANSparkMax(4, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(8);
        m_motor.setSecondaryCurrentLimit(8);
        m_motor.setIdleMode(IdleMode.kBrake);

        m_encoder = new AnalogEncoder(new AnalogInput(1));
        m_previousPosition = getPosition();
    }

    public double getPosition() {
        double result = m_filter.calculate(getMeasurement());
        t.log(Level.DEBUG, "/simple/position", result);
        return result;
    }

    public double getVelocity() {
        double position = getPosition();
        m_previousPosition = position;
        double result = (position - m_previousPosition) * 50;
        t.log(Level.DEBUG, "/simple/velocity", result);
        return result;
    }

    public void setDutyCycle(double u) {
        m_motor.set(u);
    }

    private double getMeasurement() {
        return m_encoder.getAbsolutePosition() - m_config.encoderZero;
    }

}
