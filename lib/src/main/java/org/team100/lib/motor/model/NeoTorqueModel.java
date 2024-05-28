package org.team100.lib.motor.model;

/**
 * Constants for Neo motor.
 * 
 * @see https://www.revrobotics.com/rev-21-1650/
 */
public interface NeoTorqueModel extends TorqueModel {
    @Override
    default double kROhms() {
        return 0.114;
    }

    @Override
    default double kTNm_amp() {
        return 0.028;
    }

}
