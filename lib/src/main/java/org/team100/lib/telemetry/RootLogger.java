package org.team100.lib.telemetry;

import org.team100.lib.logging.SupplierLogger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A root logger includes a boolean "enable" entry, which affects all the child
 * loggers.
 */
public class RootLogger extends SupplierLogger {

    RootLogger(
            Telemetry telemetry,
            String root,
            boolean defaultEnabledNT,
            boolean defaultEnabledUSB) {
        super(telemetry, root,
                select(root + "/enableNT", defaultEnabledNT), telemetry.ntLogger,
                select(root + "/enableUSB", defaultEnabledUSB), telemetry.usbLogger);
    }

    private static BooleanSubscriber select(
            String name,
            boolean defaultEnabled) {
        BooleanTopic t = NetworkTableInstance.getDefault().getBooleanTopic(name);
        BooleanPublisher p = t.publish();
        t.setRetained(true);
        p.set(defaultEnabled);
        return t.subscribe(defaultEnabled);
    }

}