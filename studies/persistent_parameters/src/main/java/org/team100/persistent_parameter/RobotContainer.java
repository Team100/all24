package org.team100.persistent_parameter;

import java.util.HashMap;
import java.util.Map;

public class RobotContainer {
    public RobotContainer() {
        HIDControl hid = new HIDControl();
        Map<String, Integer> k = new HashMap<>();
        k.put("foo", 0);
        k.put("bar", 1);
        k.put("baz", 2);
        k.put("biz", 3);
        ParameterFactory parameters = new ParameterFactory(hid, k);
        ExampleSubsystem s = new ExampleSubsystem(parameters);
        hid.reset(1).onTrue(s.resetA());
        hid.reset(2).onTrue(s.resetB());
    }
}
