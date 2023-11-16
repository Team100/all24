package org.team100.lib.motion.crank;

import java.util.function.Consumer;

public class CrankOutboardVelocityServo implements Consumer<CrankActuation> {
    private final MotorWrapper m_motor;

    public CrankOutboardVelocityServo(MotorWrapper motor) {
        m_motor = motor;
        // TODO: motor setup here for closed loop mode
    }

    /** Set the outboard closed-loop setpoint. */
    @Override
    public void accept(CrankActuation state) {
        double feedforward = 0.0; // TODO: real feedforward
        m_motor.setPID(state.getVelocityM_S(), feedforward);
    }

}
