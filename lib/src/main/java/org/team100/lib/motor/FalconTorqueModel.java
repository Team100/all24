package org.team100.lib.motor;

/**
 * Constants for Falcon 500.
 * 
 * @see https://store.ctr-electronics.com/content/datasheet/Motor%20Performance%20Analysis%20Report.pdf
 */
public interface FalconTorqueModel extends TorqueModel {
    @Override
    default double kROhms() {
        return 0.03;
    }

    @Override
    default double kTNm_amp() {
        return 0.018;
    }
}
