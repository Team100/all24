package org.team100.sim;

import org.dyn4j.world.ValueMixer;

/**
 * The default value mixer takes the max of the COR which is wrong; this
 * multiplies them instead.
 */
public class ValueMixer100 implements ValueMixer {

    @Override
    public double mixFriction(double friction1, double friction2) {
        return Math.sqrt(friction1 * friction2);
    }

    @Override
    public double mixRestitution(double restitution1, double restitution2) {
        return restitution1 * restitution2;
    }

    @Override
    public double mixRestitutionVelocity(double restitutionVelocity1, double restitutionVelocity2) {
        return Math.min(restitutionVelocity1, restitutionVelocity2);
    }

}
