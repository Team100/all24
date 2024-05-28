package org.team100.lib.motor.model;

/**
 * Constants for Vortex motor.
 * 
 * @see https://www.revrobotics.com/rev-21-1652/
 */
public interface NeoVortexTorqueModel extends TorqueModel {
    @Override
    default double kROhms() {
        return 0.057;
    }

    @Override
    default double kTNm_amp() {
        return 0.017;
    }

}
