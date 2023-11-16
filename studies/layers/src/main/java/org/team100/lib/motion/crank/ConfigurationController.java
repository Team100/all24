package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

public class ConfigurationController implements Supplier<Actuation> {
    private final PIDController m_controller;

    private final Supplier<Configuration> m_measurement;
    private final Supplier<Configuration> m_setpoint;

    public ConfigurationController(
            Supplier<Configuration> measurement,
            Supplier<Configuration> setpoint) {
        m_measurement = measurement;
        m_setpoint = setpoint;
        m_controller = new PIDController(1, 0, 0);
    }

    @Override
    public Actuation get() {
        return new Actuation(m_controller.calculate(
                m_measurement.get().getCrankAngleRad(),
                m_setpoint.get().getCrankAngleRad()));
    }
}
