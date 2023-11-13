package org.team100.persistent_parameter;

public class RobotContainer {
    public RobotContainer() {
        HIDControl hid = new HIDControl();
        ExampleSubsystem s = new ExampleSubsystem(hid::getKnobValue);
        hid.reset().onTrue(s.reset());
    }
}
