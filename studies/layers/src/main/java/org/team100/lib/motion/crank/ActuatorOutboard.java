package org.team100.lib.motion.crank;

public class ActuatorOutboard implements Actuator {
    private final MotorWrapper m_motor;

    public ActuatorOutboard(MotorWrapper motor) {
        m_motor = motor;
        // TODO: motor setup here for closed loop mode
    }

    /** Set the outboard closed-loop setpoint. */
    @Override
    public void accept(Actuation state) {
        double feedforward = 0.0; // TODO: real feedforward
        m_motor.setPID(state.getVelocityM_S(), feedforward);
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }

}
