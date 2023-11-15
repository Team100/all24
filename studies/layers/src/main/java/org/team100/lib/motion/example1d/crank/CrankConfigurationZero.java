package org.team100.lib.motion.example1d.crank;

/** Always produces zero actuation. */
public class CrankConfigurationZero extends CrankConfigurationController {

    @Override
    public CrankActuation calculate(CrankConfiguration measurement, CrankConfiguration setpoint) {
        return new CrankActuation(0.0);
    }
}
