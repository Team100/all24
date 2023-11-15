package org.team100.lib.motion.example1d.crank;

import edu.wpi.first.math.controller.PIDController;

public class CrankConfigurationController {
    private final PIDController m_controller;

    public CrankConfigurationController() {
        m_controller = new PIDController(1, 0, 0);
    }

    public CrankActuation calculate(
            CrankConfiguration measurement,
            CrankConfiguration setpoint) {
        return new CrankActuation(m_controller.calculate(
                measurement.getCrankAngleRad(),
                setpoint.getCrankAngleRad()));
    }
}
