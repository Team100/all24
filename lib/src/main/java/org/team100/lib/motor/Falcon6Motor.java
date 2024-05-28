package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Names;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public abstract class Falcon6Motor<T extends Measure100> implements MotorWithEncoder100<T> {
    protected final Telemetry t = Telemetry.get();
    protected final String m_name;
    protected final TalonFX m_motor;
    protected final Feedforward100 m_ff;

    protected Falcon6Motor(
            String name,
            int canId,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            PIDConstants lowLevelVelocityConstants,
            Feedforward100 ff) {
        m_name = Names.append(name, this);
        m_motor = new TalonFX(canId);
        m_ff = ff;

        TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();
        Phoenix100.baseConfig(talonFXConfigurator);
        Phoenix100.motorConfig(talonFXConfigurator, motorPhase);
        Phoenix100.currentConfig(talonFXConfigurator, supplyLimit, statorLimit);
        Phoenix100.pidConfig(talonFXConfigurator, lowLevelVelocityConstants);
        Phoenix100.crash(() -> m_motor.getVelocity().setUpdateFrequency(50));
        t.log(Level.TRACE, m_name, "Device ID", m_motor.getDeviceID());

    }

    @Override
    public void setDutyCycle(double output) {
        DutyCycleOut d = new DutyCycleOut(output);
        Phoenix100.warn(() -> m_motor.setControl(d));
        t.log(Level.TRACE, m_name, "desired duty cycle [-1,1]", output);
        log();
    }

    @Override
    public double getTorque() {
        // TODO: latency compensation
        StatusSignal<Double> statorCurrentAmpsStatus = m_motor.getTorqueCurrent();
        double statorCurrentAmps = statorCurrentAmpsStatus.getValueAsDouble();
        return statorCurrentAmps * kTNm_amp();
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void reset() {
        resetPosition();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * Sets integrated sensor position to zero.
     */
    public void resetPosition() {
        Phoenix100.warn(() -> m_motor.setPosition(0));
    }

    protected void log() {
        t.log(Level.TRACE, m_name, "velocity (rev_s)", m_motor.getVelocity().getValueAsDouble());
        t.log(Level.TRACE, m_name, "output [-1,1]", m_motor.getDutyCycle().getValueAsDouble());
        t.log(Level.TRACE, m_name, "error (rev_s)", m_motor.getClosedLoopError().getValueAsDouble());
        t.log(Level.TRACE, m_name, "supply current (A)", m_motor.getSupplyCurrent().getValueAsDouble());
        t.log(Level.TRACE, m_name, "stator current (A)", m_motor.getStatorCurrent().getValueAsDouble());
        t.log(Level.DEBUG, m_name, "temperature (C)", m_motor.getDeviceTemp().getValueAsDouble());
    }

    /**
     * Motor resistance in ohms, used to calculate voltage from desired torque
     * current.
     */
    protected abstract double kROhms();

    /**
     * Motor torque constant, kT, in Nm per amp, used to calculate current from
     * desired torque.
     */
    protected abstract double kTNm_amp();

}
