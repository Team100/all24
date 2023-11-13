package org.team100.persistent_parameter;

import java.util.HashMap;
import java.util.Map;

public class RobotContainer {
    ExampleSubsystem m_subsystem;

    public RobotContainer() {
        HIDControl hid = new HIDControl();
        Map<String, PersistentParameter.HIDConfig> configs = new HashMap<>();
        configs.put("foo", conf(hid, 0, 1));
        configs.put("bar", conf(hid, 1, 2));
        configs.put("baz", conf(hid, null, 3)); // no knob
        configs.put("biz", conf(hid, 3, null)); // no reset
        ParameterFactory parameters = new ParameterFactory(configs);
        m_subsystem = new ExampleSubsystem(parameters);
    }

    public void doNothing() {
        m_subsystem.doNothing();
    }

    private PersistentParameter.HIDConfig conf(HIDControl hid, Integer knob, Integer button) {
        return new PersistentParameter.HIDConfig(
                knob == null ? () -> 0.0 : () -> hid.knob(knob),
                button == null ? () -> false : () -> hid.button(button));
    }
}
