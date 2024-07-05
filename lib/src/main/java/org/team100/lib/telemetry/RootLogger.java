package org.team100.lib.telemetry;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;

/**
 * A root logger includes a boolean "enable" entry, which affects all the child
 * loggers.
 */
public class RootLogger extends NTLogger {

    RootLogger(Telemetry telemetry, String root, boolean defaultEnabled) {
        super(telemetry, root, getT(telemetry, root, defaultEnabled));
    }

    private static BooleanSubscriber getT(
            Telemetry telemetry,
            String root,
            boolean defaultEnabled) {
        BooleanTopic t = telemetry.inst.getBooleanTopic(root + "/enable");
        BooleanPublisher p = t.publish();
        t.setRetained(true);
        p.set(defaultEnabled);
        return t.subscribe(defaultEnabled);
    }

}