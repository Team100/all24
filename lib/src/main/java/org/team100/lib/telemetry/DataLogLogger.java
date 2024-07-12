package org.team100.lib.telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.function.FloatSupplier;

/** An attempt to reduce logging CPU load by using DataLog */
public class DataLogLogger implements Logger {
    private final Telemetry m_telemetry;
    private final String m_root;
    private final BooleanSupplier m_enabled;

    public DataLogLogger(Telemetry telemetry, String root, BooleanSupplier enabled) {
        m_telemetry = telemetry;
        m_root = root;
        m_enabled = enabled;
    }

    @Override
    public Logger child(String stem) {
        return new DataLogLogger(m_telemetry, m_root + "/" + stem, m_enabled);
    }

    @Override
    public void logBoolean(Level level, String leaf, BooleanSupplier val) {
        if (!allow(level))
            return;
        pub(
                append(m_root, leaf),
                k -> new BooleanLogEntry(m_telemetry.m_log, k),
                BooleanLogEntry.class).append(val.getAsBoolean());
    }

    @Override
    public void logDouble(Level level, String leaf, DoubleSupplier vals) {
        if (!allow(level))
            return;
        pub(
                append(m_root, leaf),
                k -> new DoubleLogEntry(m_telemetry.m_log, k),
                DoubleLogEntry.class).append(vals.getAsDouble());
    }

    @Override
    public void logInt(Level level, String leaf, IntSupplier vals) {
        if (!allow(level))
            return;
        pub(
                append(m_root, leaf),
                k -> new IntegerLogEntry(m_telemetry.m_log, k),
                IntegerLogEntry.class).append(vals.getAsInt());
    }

    @Override
    public void logFloat(Level level, String leaf, FloatSupplier val) {
        if (!allow(level))
            return;
        pub(
                append(m_root, leaf),
                k -> new FloatLogEntry(m_telemetry.m_log, k),
                FloatLogEntry.class).append(val.getAsFloat());
    }

    @Override
    public void logDoubleArray(Level level, String leaf, Supplier<double[]> val) {
        if (!allow(level))
            return;
        pub(
                append(m_root, leaf),
                k -> new DoubleArrayLogEntry(m_telemetry.m_log, k),
                DoubleArrayLogEntry.class).append(val.get());
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
        pub(
                append(m_root, leaf),
                k -> new IntegerLogEntry(m_telemetry.m_log, k),
                IntegerLogEntry.class).append(vals.getAsLong());
    }

    @Override
    public void logString(Level level, String leaf, Supplier<String> vals) {
        if (!allow(level))
            return;
        pub(
                append(m_root, leaf),
                k -> new StringLogEntry(m_telemetry.m_log, k),
                StringLogEntry.class).append(vals.get());
    }

    @Override
    public void logStringArray(Level level, String leaf, Supplier<String[]> vals) {
        if (!allow(level))
            return;
        pub(
                append(m_root, leaf),
                k -> new StringArrayLogEntry(m_telemetry.m_log, k),
                StringArrayLogEntry.class).append(vals.get());
    }

    ///////////////////////////////////////////////////////////

    private <T extends DataLogEntry> T pub(String key, Function<String, DataLogEntry> fn, Class<T> entryClass) {
        DataLogEntry entry = m_telemetry.entries.computeIfAbsent(valid(key), fn);
        if (!entryClass.isInstance(entry))
            throw new IllegalArgumentException(
                    String.format("value type clash for key %s old %s new %s",
                            key,
                            entry.getClass().getName(),
                            entryClass.getName()));
        return entryClass.cast(entry);
    }

    private String valid(String key) {
        if (key.length() == 0)
            throw new IllegalArgumentException("empty key");
        if (key.charAt(0) != '/')
            throw new IllegalArgumentException("key must start with slash; you provided: " + key);
        return key;
    }

    private boolean allow(Level level) {
        if (m_telemetry.m_level == Level.COMP && level == Level.COMP) {
            // comp mode allows COMP level regardless of enablement.
            return true;
        }
        return enabled() && m_telemetry.m_level.admit(level);
    }

    private boolean enabled() {
        return m_enabled.getAsBoolean();
    }
}
