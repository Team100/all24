package org.team100.lib.motor;

/**
 * Motor constants used for torque calculations.
 */
public interface TorqueModel {
    /**
     * Motor resistance in ohms, used to calculate voltage from desired torque
     * current.
     */
    double kROhms();

    /**
     * Motor torque constant, kT, in Nm per amp, used to calculate current from
     * desired torque.
     */
    double kTNm_amp();

}
