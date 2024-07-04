package org.team100.lib.telemetry;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;

/**
 * A root logger includes a boolean "enable" entry, which affects all the child
 * loggers.
 */
public class RootLogger extends NTLogger {

    RootLogger(Telemetry telemetry, String root) {
        super(telemetry, root, getT(telemetry, root));
    }

    private static BooleanSubscriber getT(Telemetry telemetry, String root) {
        BooleanTopic t = telemetry.inst.getBooleanTopic(root + "/enable");
        t.publish().set(false);
        t.getEntry(false);
        t.setRetained(true);
        return t.subscribe(false);
    }

}