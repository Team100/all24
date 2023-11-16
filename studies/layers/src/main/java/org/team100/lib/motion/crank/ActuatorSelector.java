package org.team100.lib.motion.crank;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.Consumer;

/** Select and indicate an actuator. */
public class ActuatorSelector implements Consumer<Actuation> {
    public enum Actuator {
        ONBOARD, OUTBOARD
    }

    private final Map<Actuator, Consumer<Actuation>> m_actuators;
    private Actuator m_actuator;

    public ActuatorSelector(MotorWrapper motor) {
        m_actuators = new EnumMap<>(Actuator.class);
        m_actuators.put(Actuator.ONBOARD, new ActuatorOnboard(motor));
        m_actuators.put(Actuator.OUTBOARD, new ActuatorOutboard(motor));
    }

    @Override
    public void accept(Actuation state) {
        if (m_actuator == null) return;
        Consumer<Actuation> choice = m_actuators.get(m_actuator);
        choice.accept(state);
    }

    public void set(Actuator actuator) {
        m_actuator = actuator;
    }

    public Actuator get() {
        return m_actuator;
    }
}
