package org.team100.lib.motor.model;

/**
 * Constants for Kraken.
 * 
 * @see https://store.ctr-electronics.com/content/datasheet/Motor%20Performance%20Analysis%20Report.pdf
 */
public interface KrakenTorqueModel extends TorqueModel {
    @Override
    default double kROhms() {
        return 0.025;
    }

    @Override
    default double kTNm_amp() {
        return 0.019;
    }
}
