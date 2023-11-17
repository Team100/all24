package org.team100.lib.motion.crank;

/** Swallows all input. */
public class ActuatorNull implements Actuator {
    @Override
    public void accept(Actuation state) {
        System.out.println(state);
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }

}
