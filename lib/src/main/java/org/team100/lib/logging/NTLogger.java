package org.team100.lib.logging;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Function;
import java.util.stream.Stream;

import org.team100.lib.telemetry.Chronos;
import org.team100.lib.telemetry.Chronos.Sample;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

public class NTLogger extends PrimitiveLogger {
    private final NetworkTableInstance inst;
    private final Map<String, Publisher> m_publishers;
    private final Chronos m_chronos;

    public NTLogger() {
        inst = NetworkTableInstance.getDefault();
        m_publishers = new ConcurrentHashMap<>();
        m_chronos = Chronos.get();
    }

    @Override
    public void logBoolean(String key, boolean val) {
        pub(key, k -> {
            BooleanTopic t = inst.getBooleanTopic(k);
            BooleanPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, BooleanPublisher.class).set(val);
    }

    @Override
    public void logDouble(String key, double val) {
        Sample s = m_chronos.sample("NTLogger/logDouble");
        try {
            pub(key, k -> {
                DoubleTopic t = inst.getDoubleTopic(k);
                DoublePublisher p = t.publish();
                t.setRetained(true);
                return p;
            }, DoublePublisher.class).set(val);
        } finally {
            s.end();
        }
    }

    @Override
    public void logInt(String key, int val) {
        pub(key, k -> {
            IntegerTopic t = inst.getIntegerTopic(k);
            IntegerPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, IntegerPublisher.class).set(val);
    }

    @Override
    public void logDoubleArray(String key, double[] val) {
        pub(key, k -> {
            DoubleArrayTopic t = inst.getDoubleArrayTopic(k);
            DoubleArrayPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, DoubleArrayPublisher.class).set(val);
    }

    @Override
    public void logDoubleObjArray(String key, Double[] val) {
        logDoubleArray(key, Stream.of(val).mapToDouble(Double::doubleValue).toArray());
    }

    @Override
    public void logLong(String key, long val) {
        pub(key, k -> {
            IntegerTopic t = inst.getIntegerTopic(k);
            IntegerPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, IntegerPublisher.class).set(val);
    }

    @Override
    public void logString(String key, String val) {
        pub(key, k -> {
            StringTopic t = inst.getStringTopic(k);
            StringPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, StringPublisher.class).set(val);
    }

    private <T extends Publisher> T pub(String key, Function<String, Publisher> fn, Class<T> pubClass) {
        Publisher publisher = m_publishers.computeIfAbsent(valid(key), fn);
        if (!pubClass.isInstance(publisher))
            throw new IllegalArgumentException(
                    String.format("value type clash for key %s old %s new %s",
                            key,
                            publisher.getClass().getName(),
                            pubClass.getName()));
        return pubClass.cast(publisher);
    }

    private String valid(String key) {
        if (key.length() == 0)
            throw new IllegalArgumentException("empty key");
        if (key.charAt(0) != '/')
            throw new IllegalArgumentException("key must start with slash; you provided: " + key);
        return key;
    }

}