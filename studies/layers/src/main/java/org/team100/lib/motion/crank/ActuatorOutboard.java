package org.team100.lib.motion.crank;

import java.util.function.Consumer;

public class ActuatorOutboard implements Consumer<Actuation> {
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

}
