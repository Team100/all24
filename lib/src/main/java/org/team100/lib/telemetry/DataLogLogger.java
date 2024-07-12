package org.team100.lib.telemetry;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Function;
import java.util.stream.Stream;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/** An attempt to reduce logging CPU load by using DataLog */
public class DataLogLogger extends PrimitiveLogger {
    private final DataLog m_log;
    private final Map<String, DataLogEntry> entries;

    public DataLogLogger() {
        // create a log file but don't write network tables to it
        DataLogManager.logNetworkTables(false);
        m_log = DataLogManager.getLog();
        
        entries = new ConcurrentHashMap<>();
    }

    @Override
    public void logBoolean(String key, boolean val) {
        pub(key,
                k -> new BooleanLogEntry(m_log, k),
                BooleanLogEntry.class).append(val);
    }

    @Override
    public void logDouble(String key, double val) {
        pub(key,
                k -> new DoubleLogEntry(m_log, k),
                DoubleLogEntry.class).append(val);
    }

    @Override
    public void logInt(String key, int val) {
        pub(key,
                k -> new IntegerLogEntry(m_log, k),
                IntegerLogEntry.class).append(val);
    }

    @Override
    public void logFloat(String key, float val) {
        pub(key,
                k -> new FloatLogEntry(m_log, k),
                FloatLogEntry.class).append(val);
    }

    @Override
    public void logDoubleArray(String key, double[] val) {
        pub(key,
                k -> new DoubleArrayLogEntry(m_log, k),
                DoubleArrayLogEntry.class).append(val);
    }

    @Override
    public void logDoubleObjArray(String key, Double[] val) {
        logDoubleArray(key, Stream.of(val).mapToDouble(Double::doubleValue).toArray());
    }

    @Override
    public void logLong(String key, long val) {
        pub(key,
                k -> new IntegerLogEntry(m_log, k),
                IntegerLogEntry.class).append(val);
    }

    @Override
    public void logString(String key, String val) {
        pub(key,
                k -> new StringLogEntry(m_log, k),
                StringLogEntry.class).append(val);
    }

    @Override
    public void logStringArray(String key, String[] val) {
        pub(key,
                k -> new StringArrayLogEntry(m_log, k),
                StringArrayLogEntry.class).append(val);
    }

    ///////////////////////////////////////////////////////////

    private <T extends DataLogEntry> T pub(String key, Function<String, DataLogEntry> fn, Class<T> entryClass) {
        DataLogEntry entry = entries.computeIfAbsent(valid(key), fn);
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

}
