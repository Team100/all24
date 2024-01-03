package org.team100.lib.telemetry;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * Turns on a digital output in response to boolean alerts, e.g. from the
 * Monitor class.
 */
public class Annunciator implements BooleanConsumer {
    private final DigitalOutput m_pwm;
    private final AnnunciatorVisualization m_viz;

    public Annunciator(int channel) {
        m_pwm = new DigitalOutput(channel);
        m_viz = new AnnunciatorVisualization();
    }

    @Override
    public void accept(boolean value) {
        m_pwm.set(value);
        m_viz.set(value);
    }
}
