package org.team100.lib.motion.crank;

public class ActuatorOnboard implements Actuator {
    // package private for testing.
    final MotorWrapper m_motor;

    public ActuatorOnboard(MotorWrapper motor) {
        m_motor = motor;
        // TODO: motor setup here for duty cycle mode
    }

    /** Set the duty cycle. */
    @Override
    public void accept(Actuation state) {
        double kV = 1.0; // TODO: real motor model
        double u_FF = kV * state.getVelocityM_S();
        double u_FB = 0.0; // TODO: feedback control
        m_motor.set(u_FF + u_FB);
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }
}
