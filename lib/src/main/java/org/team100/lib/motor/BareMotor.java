package org.team100.lib.motor;

import org.team100.lib.dashboard.Glassy;

/**
 * A bare motor is like a Motor100 but the methods pertain only to its output
 * shaft, not the motion of the attached mechanism. Accordingly, the units are
 * always rotational, and there should be no gear ratios in any implementation.
 */
public interface BareMotor extends Glassy {

    /**
     * Some motors allow torque limiting through current limiting.
     */
    void setTorqueLimit(double torqueNm);

    /**
     * Open-loop duty cycle control.
     * 
     * @param output in range [-1, 1]
     */
    void setDutyCycle(double output);

    /**
     * Velocity feedback with friction, velocity, acceleration, and holding torque.
     * 
     * Could be open-loop (e.g. "kV") or closed-loop.
     * 
     * @param velocityRad_S motor shaft speed, rad/s.
     * @param accelRad_S2   rad/s^2.
     * @param torqueNm      Nm, for gravity compensation or holding.
     */
    void setVelocity(
            double velocityRad_S,
            double accelRad_S2,
            double torqueNm);

    /**
     * Should be cached.
     * 
     * Motor shaft speed.
     */
    double getVelocityRad_S();

    void setEncoderPositionRad(double positionRad);

    /**
     * Position feedback with friction and velocity feedforward, and holding torque.
     * 
     * Revolutions wind up; 0 != 2pi.
     * 
     * @param positionRad   radians.
     * @param velocityRad_S rad/s.
     * @param torqueNm      Nm, for gravity compensation or holding.
     */
    void setPosition(
            double positionRad,
            double velocityRad_S,
            double torqueNm);

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

    /**
     * Incremental voltage required to produce the given torque, used for
     * feedforward.
     */
    default double getTorqueFFVolts(double torqueNm) {
        double torqueFFAmps = torqueNm / kTNm_amp();
        return torqueFFAmps * kROhms();
    }

    void stop();

    /**
     * For test cleanup.
     */
    void close();

    /** For logging */
    void periodic();

    @Override
    default String getGlassName() {
        return "BareMotor";
    }

}
