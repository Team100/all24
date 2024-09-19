package org.team100.logging;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.team100.lib.telemetry.UdpType;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;

/**
 * All the publishers and log entries.
 * The meta reader adds new entries here (in its own thread), and the data
 * reader adds items (in its own thread), so we use ConcurrentHashMap.
 */
public class UdpConsumers implements UdpConsumersInterface {
    // write to network tables
    private static final boolean PUB = false;

    // write to disk
    private static final boolean LOG = false;

    NetworkTableInstance inst;
    DataLog log_file;

    Map<Integer, BooleanPublisher> booleanPublishers = new ConcurrentHashMap<>();
    Map<Integer, BooleanLogEntry> booleanEntries = new ConcurrentHashMap<>();
    Map<Integer, DoublePublisher> doublePublishers = new ConcurrentHashMap<>();
    Map<Integer, DoubleLogEntry> doubleEntries = new ConcurrentHashMap<>();
    Map<Integer, IntegerPublisher> intPublishers = new ConcurrentHashMap<>();
    Map<Integer, IntegerLogEntry> intEntries = new ConcurrentHashMap<>();
    Map<Integer, DoubleArrayPublisher> doubleArrayPublishers = new ConcurrentHashMap<>();
    Map<Integer, DoubleArrayLogEntry> doubleArrayEntries = new ConcurrentHashMap<>();
    Map<Integer, StringPublisher> stringPublishers = new ConcurrentHashMap<>();
    Map<Integer, StringLogEntry> stringEntries = new ConcurrentHashMap<>();

    public UdpConsumers() {
        if (PUB) {
            inst = NetworkTableInstance.getDefault();
            inst.startServer();
        }
        if (LOG) {
            // log_file = DataLog(dir=LOG_DIR, filename=LOG_FILENAME)
            log_file = new DataLog("", "", 0.1);
        }
    }

    @Override
    public void acceptBoolean(int key, boolean val) {
        if (PUB) {
            BooleanPublisher pub = booleanPublishers.get(key);
            if (pub != null)
                pub.set(val);
        }
        if (LOG) {
            BooleanLogEntry entry = booleanEntries.get(key);
            if (entry != null)
                entry.append(val);
        }
    }

    @Override
    public void acceptDouble(int key, double val) {
        if (PUB) {
            DoublePublisher pub = doublePublishers.get(key);
            if (pub != null)
                pub.set(val);
        }
        if (LOG) {
            DoubleLogEntry entry = doubleEntries.get(key);
            if (entry != null)
                entry.append(val);
        }
    }

    @Override
    public void acceptInt(int key, int val) {
        if (PUB) {
            IntegerPublisher pub = intPublishers.get(key);
            if (pub != null)
                pub.set(val);
        }
        if (LOG) {
            IntegerLogEntry entry = intEntries.get(key);
            if (entry != null)
                entry.append(val);
        }
    }

    @Override
    public void acceptDoubleArray(int key, double[] val) {
        if (PUB) {
            DoubleArrayPublisher pub = doubleArrayPublishers.get(key);
            if (pub != null)
                pub.set(val);
        }
        if (LOG) {
            DoubleArrayLogEntry entry = doubleArrayEntries.get(key);
            if (entry != null)
                entry.append(val);
        }
    }

    @Override
    public void acceptString(int key, String val) {
        if (PUB) {
            StringPublisher pub = stringPublishers.get(key);
            if (pub != null)
                pub.set(val);
        }
        if (LOG) {
            StringLogEntry entry = stringEntries.get(key);
            if (entry != null)
                entry.append(val);
        }
    }

    @Override
    public void acceptMeta(int key, UdpType type, String val) {
        if (PUB) {
            switch (type) {
                case BOOLEAN -> {
                    booleanPublishers.computeIfAbsent(key, k -> {
                        var t = inst.getBooleanTopic(val);
                        var p = t.publish(PubSubOption.keepDuplicates(true));
                        t.setRetained(true);
                        return p;
                    });
                }
                case DOUBLE -> {
                    doublePublishers.computeIfAbsent(key, k -> {
                        var t = inst.getDoubleTopic(val);
                        var p = t.publish(PubSubOption.keepDuplicates(true));
                        t.setRetained(true);
                        return p;
                    });
                }
                case INT, LONG -> {
                    intPublishers.computeIfAbsent(key, k -> {
                        var t = inst.getIntegerTopic(val);
                        var p = t.publish(PubSubOption.keepDuplicates(true));
                        t.setRetained(true);
                        return p;
                    });
                }
                case DOUBLE_ARRAY -> {
                    doubleArrayPublishers.computeIfAbsent(key, k -> {
                        var t = inst.getDoubleArrayTopic(val);
                        var p = t.publish(PubSubOption.keepDuplicates(true));
                        t.setRetained(true);
                        return p;
                    });
                }
                case STRING -> {
                    stringPublishers.computeIfAbsent(key, k -> {
                        var t = inst.getStringTopic(val);
                        var p = t.publish(PubSubOption.keepDuplicates(true));
                        t.setRetained(true);
                        return p;
                    });
                }
                default -> {
                    System.out.println("unknown meta type 1");
                }
            }
        }
        if (LOG) {
            switch (type) {
                case BOOLEAN -> booleanEntries.computeIfAbsent(key,
                        k -> new BooleanLogEntry(log_file, val));
                case DOUBLE -> doubleEntries.computeIfAbsent(key,
                        k -> new DoubleLogEntry(log_file, val));
                case INT, LONG -> intEntries.computeIfAbsent(key,
                        k -> new IntegerLogEntry(log_file, val));
                case DOUBLE_ARRAY -> doubleArrayEntries.computeIfAbsent(key,
                        k -> new DoubleArrayLogEntry(log_file, val));
                case STRING -> stringEntries.computeIfAbsent(key,
                        k -> new StringLogEntry(log_file, val));
                default -> {
                    System.out.println("unknown meta type 2");
                }
            }
        }
    }
}
