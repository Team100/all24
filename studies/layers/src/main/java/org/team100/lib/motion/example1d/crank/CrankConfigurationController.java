package org.team100.lib.motion.example1d.crank;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

public class CrankConfigurationController implements Supplier<CrankActuation> {
    private final PIDController m_controller;

    private final Supplier<CrankConfiguration> m_measurement;
    private final Supplier<CrankConfiguration> m_setpoint;

    public CrankConfigurationController(
            Supplier<CrankConfiguration> measurement,
            Supplier<CrankConfiguration> setpoint) {
        m_measurement = measurement;
        m_setpoint = setpoint;
        m_controller = new PIDController(1, 0, 0);
    }

    public CrankActuation calculate(
            CrankConfiguration measurement,
            CrankConfiguration setpoint) {
        return new CrankActuation(m_controller.calculate(
                measurement.getCrankAngleRad(),
                setpoint.getCrankAngleRad()));
    }

    @Override
    public CrankActuation get() {
        return calculate(m_measurement.get(), m_setpoint.get());
    }
}
