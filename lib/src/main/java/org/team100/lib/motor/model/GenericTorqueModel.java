package org.team100.lib.motor.model;

/**
 * Placeholder for motors not yet measured.
 */
public interface GenericTorqueModel extends TorqueModel {
    @Override
    default double kROhms() {
        return 0.1;
    }

    @Override
    default double kTNm_amp() {
        return 0.02;
    }

}
