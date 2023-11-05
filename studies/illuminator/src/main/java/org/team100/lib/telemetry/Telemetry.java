package org.team100.lib.telemetry;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Simple logging wrapper.
 * 
 * Use keys of the form "/foo/bar"; the slashes separate levels in the tree.
 * 
 * Don't use a slash for any other reason, e.g. for "meters per second" don't say "m/s" say "m_s"
 */
public class Telemetry {
    private static final Telemetry instance = new Telemetry();
    private final NetworkTableInstance inst;
    private final Map<String, Publisher> pubs;

    /** Uses the default network table instance. */
    public Telemetry() {
        inst = NetworkTableInstance.getDefault();
        pubs = new HashMap<>();
        DataLogManager.start();
    }

    public static Telemetry get() {
        return instance;
    }

    public void log(String key, boolean val) {
        pub(key, k -> inst.getBooleanTopic(k).publish(), BooleanPublisher.class).set(val);
    }

    public void log(String key, double val) {
        pub(key, k -> inst.getDoubleTopic(k).publish(), DoublePublisher.class).set(val);
    }

    public void log(String key, double[] val) {
        pub(key, k -> inst.getDoubleArrayTopic(k).publish(), DoubleArrayPublisher.class).set(val);
    }

    public void log(String key, long val) {
        pub(key, k -> inst.getIntegerTopic(k).publish(), IntegerPublisher.class).set(val);
    }

    public void log(String key, String val) {
        pub(key, k -> inst.getStringTopic(k).publish(), StringPublisher.class).set(val);
    }

    public void log(String key, String[] val) {
        pub(key, k -> inst.getStringArrayTopic(k).publish(), StringArrayPublisher.class).set(val);
    }

    private <T extends Publisher> T pub(String key, Function<String, Publisher> fn, Class<T> pubClass) {
        Publisher publisher = pubs.computeIfAbsent(valid(key), fn);
        if (!pubClass.isInstance(publisher))
            throw new IllegalArgumentException("value type clash");
        return pubClass.cast(publisher);
    }

    private String valid(String key) {
        if (key.length() == 0)
            throw new IllegalArgumentException("empty key");
        if (key.charAt(0) != '/')
            throw new IllegalArgumentException("no leading slash");
        return key;
    }

    /** For testing. */
    Telemetry(NetworkTableInstance inst, Map<String, Publisher> pubs) {
        this.inst = inst;
        this.pubs = pubs;
    }
}
