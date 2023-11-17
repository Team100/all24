package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

public class ConfigurationController implements Actuations {
    private final PIDController m_controller;
    private final Supplier<Configurations> m_measurement;
    private final Supplier<Configurations> m_setpoint;

    /** PID in configuration space */
    public ConfigurationController(
        Supplier<Configurations> measurement,
        Supplier<Configurations> setpoint) {
        m_measurement = measurement;
        m_setpoint = setpoint;
        m_controller = new PIDController(1, 0, 0);
    }

    @Override
    public Actuation get() {
        return new Actuation(m_controller.calculate(
                m_measurement.get().get().getCrankAngleRad(),
                m_setpoint.get().get().getCrankAngleRad()));
    }

    @Override
    public void accept(Indicator indicator) {
        m_measurement.get().accept(indicator);
        m_setpoint.get().accept(indicator);
        indicator.indicate(this);
    }
}
