package org.team100.persistent_parameter;

public class RobotContainer {
    public RobotContainer() {
        HIDControl hid = new HIDControl();
        ParameterFactory parameters = new ParameterFactory(hid);
        ExampleSubsystem s = new ExampleSubsystem(parameters);
        hid.reset().onTrue(s.reset());
    }
}
