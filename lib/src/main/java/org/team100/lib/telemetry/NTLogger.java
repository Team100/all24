package org.team100.lib.telemetry;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.team100.lib.telemetry.Chronos.Sample;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.FloatTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.function.FloatSupplier;

public class NTLogger implements Logger {
    private final Telemetry m_telemetry;
    private final String m_root;
    private final BooleanSupplier m_enabled;
    private final Chronos m_chronos;

    NTLogger(Telemetry telemetry, String root, BooleanSupplier enabled) {
        m_telemetry = telemetry;
        m_root = root;
        m_enabled = enabled;
        m_chronos = Chronos.get();
    }

    @Override
    public Logger child(String stem) {
        return new NTLogger(m_telemetry, m_root + "/" + stem, m_enabled);
    }

    private boolean enabled() {
        return m_enabled.getAsBoolean();
    }

    private boolean allow(Level level) {
        if (m_telemetry.m_level == Level.COMP && level == Level.COMP) {
            // comp mode allows COMP level regardless of enablement.
            return true;
        }
        return enabled() && m_telemetry.m_level.admit(level);
    }

    @Override
    public void logBoolean(Level level, String leaf, BooleanSupplier vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        boolean val = vals.getAsBoolean();
        pub(key, k -> {
            BooleanTopic t = m_telemetry.inst.getBooleanTopic(k);
            BooleanPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, BooleanPublisher.class).set(val);
    }

    @Override
    public void logDouble(Level level, String leaf, DoubleSupplier vals) {
        Sample s = m_chronos.sample("logDouble");
        try {
            if (!allow(level))
                return;
            String key = append(m_root, leaf);
            double val = vals.getAsDouble();
            pub(key, k -> {
                DoubleTopic t = m_telemetry.inst.getDoubleTopic(k);
                DoublePublisher p = t.publish();
                t.setRetained(true);
                return p;
            }, DoublePublisher.class).set(val);
        } finally {
            s.end();
        }
    }

    @Override
    public void logInt(Level level, String leaf, IntSupplier vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        int val = vals.getAsInt();
        pub(key, k -> {
            IntegerTopic t = m_telemetry.inst.getIntegerTopic(k);
            IntegerPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, IntegerPublisher.class).set(val);
    }

    @Override
    public void logFloat(Level level, String leaf, FloatSupplier vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        float val = vals.getAsFloat();
        pub(key, k -> {
            FloatTopic t = m_telemetry.inst.getFloatTopic(k);
            FloatPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, FloatPublisher.class).set(val);
    }

    @Override
    public void logDoubleArray(Level level, String leaf, Supplier<double[]> vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        double[] val = vals.get();
        pub(key, k -> {
            DoubleArrayTopic t = m_telemetry.inst.getDoubleArrayTopic(k);
            DoubleArrayPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, DoubleArrayPublisher.class).set(val);
    }

    @Override
    public void logDoubleObjArray(Level level, String leaf, Supplier<Double[]> vals) {
        if (!allow(level))
            return;
        Double[] val = vals.get();
        logDoubleArray(level, leaf, () -> Stream.of(val).mapToDouble(Double::doubleValue).toArray());
    }

    @Override
    public void logLong(Level level, String leaf, LongSupplier vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        long val = vals.getAsLong();
        pub(key, k -> {
            IntegerTopic t = m_telemetry.inst.getIntegerTopic(k);
            IntegerPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, IntegerPublisher.class).set(val);
    }

    @Override
    public void logString(Level level, String leaf, Supplier<String> vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        String val = vals.get();
        pub(key, k -> {
            StringTopic t = m_telemetry.inst.getStringTopic(k);
            StringPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, StringPublisher.class).set(val);
    }

    /** val is a supplier to avoid doing any work if we're not going to log it. */
    @Override
    public void logStringArray(Level level, String leaf, Supplier<String[]> vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        String[] val = vals.get();
        pub(key, k -> {
            StringArrayTopic t = m_telemetry.inst.getStringArrayTopic(k);
            StringArrayPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, StringArrayPublisher.class).set(val);
    }

    private <T extends Publisher> T pub(String key, Function<String, Publisher> fn, Class<T> pubClass) {
        Publisher publisher = m_telemetry.pubs.computeIfAbsent(valid(key), fn);
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