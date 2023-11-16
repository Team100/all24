package org.team100.lib.motion.example1d.crank;

/** Always produces zero actuation. */
// TODO remove this superclass
public class CrankConfigurationZero extends CrankConfigurationController {

    public CrankConfigurationZero() {
        super(null,null);
    }

    @Override
    public CrankActuation calculate(CrankConfiguration measurement, CrankConfiguration setpoint) {
        return new CrankActuation(0.0);
    }
}
