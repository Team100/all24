package org.team100.lib.motion.example1d.sled;

import org.team100.lib.motion.example1d.framework.ConfigurationController;

import edu.wpi.first.math.controller.PIDController;

// this uses pid.
// TODO: make several of these
public class SledConfigurationController implements ConfigurationController<SledConfiguration, SledActuation> {

    private final PIDController m_controller;

    public SledConfigurationController() {
        m_controller = new PIDController(1, 0, 0);
    }

    @Override
    public SledActuation calculate(
        SledConfiguration measurement, 
    SledConfiguration setpoint) {
        return new SledActuation(m_controller.calculate(
            measurement.getPositionM(),
            setpoint.getPositionM()));
    }

}
