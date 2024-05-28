package org.team100.lib.motor.model;

/**
 * Motor constants used for torque calculations.
 * 
 * Child interfaces should implement defaults representing specific motors.
 * 
 * These child interfaces should be used as mix-ins for specific motor classes.
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
