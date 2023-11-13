package org.team100.lib.motion.example1d.crank;

import org.team100.lib.motion.example1d.framework.ConfigurationController;

/** Always produces zero actuation. */
public class CrankConfigurationZero implements ConfigurationController<CrankConfiguration, CrankActuation> {

    @Override
    public CrankActuation calculate(CrankConfiguration measurement, CrankConfiguration setpoint) {
        return new CrankActuation(0.0);
    }
}
