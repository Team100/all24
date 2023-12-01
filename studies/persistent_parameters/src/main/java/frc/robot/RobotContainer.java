package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.team100.ExampleSubsystem;
import org.team100.lib.persistent_parameter.HIDControl;
import org.team100.lib.persistent_parameter.ParameterFactory;
import org.team100.lib.persistent_parameter.PersistentParameter;

import edu.wpi.first.wpilibj2.command.PrintCommand;

public class RobotContainer {
    ExampleSubsystem m_subsystem;

    public RobotContainer() {
        HIDControl hid = new HIDControl();
        Map<String, PersistentParameter.HIDConfig> configs = new HashMap<>();
        // These id's match the "knobs" hardware that identifies as "Team 100 Knobs."
        // The code can be found in all24/studies/console/arduino/knobs
        // each encoder has a "button" that resets it
        // WPI counts buttons from 0 and axes from 1.
        configs.put("foo", conf(hid, 0, 1));
        configs.put("bar", conf(hid, 1, 2));
        configs.put("baz", conf(hid, null, 3)); // no knob, but reset works.
        configs.put("biz", conf(hid, 4, null)); // no reset, but knob works.
        ParameterFactory parameters = new ParameterFactory(configs);
        m_subsystem = new ExampleSubsystem(parameters);
        // the reset event can also trigger commands.
        hid.reset(1).onTrue(new PrintCommand("Reset one"));
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
