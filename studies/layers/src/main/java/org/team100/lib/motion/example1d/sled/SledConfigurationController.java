package org.team100.lib.motion.example1d.sled;

import edu.wpi.first.math.controller.PIDController;

// this uses pid.
// TODO: make several of these
public class SledConfigurationController {

    private final PIDController m_controller;

    public SledConfigurationController() {
        m_controller = new PIDController(1, 0, 0);
    }

    public SledActuation calculate(
            SledConfiguration measurement,
            SledConfiguration setpoint) {
        return new SledActuation(m_controller.calculate(
                measurement.getPositionM(),
                setpoint.getPositionM()));
    }
}
