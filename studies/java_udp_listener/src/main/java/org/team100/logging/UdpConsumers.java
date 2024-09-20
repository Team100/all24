package org.team100.logging;

import static java.util.concurrent.TimeUnit.SECONDS;

import java.time.Instant;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.atomic.AtomicInteger;

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
    // see DataLogManager.java
    private static final ZoneId m_utc = ZoneId.of("UTC");
    private static final DateTimeFormatter m_timeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss")
            .withZone(m_utc);
    // write to network tables
    private static final boolean PUB = true;

    // write to disk
    private static final boolean LOG = true;

    // write the count periodically
    private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
    private final AtomicInteger counter = new AtomicInteger(0);

    NetworkTableInstance inst;
    DataLog log_file;

    volatile long m_timestamp;

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
        scheduler.scheduleAtFixedRate(
                () -> System.out.printf("counter %d\n", counter.getAndSet(0)),
                0, 1, SECONDS);
        if (PUB) {
            // inst = NetworkTableInstance.getDefault();
            // inst.startServer();
        }
        if (LOG) {
            // log_file = DataLog(dir=LOG_DIR, filename=LOG_FILENAME)
            // log_file = new DataLog("", "", 0.1);
        }
    }

    @Override
    public boolean validateTimestamp(long timestamp) {
        if (timestamp == 0) {
            System.out.println("zero timestamp");
            // not a real timestamp, this only happens on startup
            // before the DS connects to the robot. Since we don't
            // know what the timestamp is, there's no reason to record
            // any data that arrives.
            m_timestamp = 0;
            return true;
        }
        if (m_timestamp == 0 || timestamp != m_timestamp) {
            System.out.println("new timestamp");
            m_timestamp = 0;
            
            booleanPublishers.clear();
            booleanEntries.clear();
            doublePublishers.clear();
            doubleEntries.clear();
            intPublishers.clear();
            intEntries.clear();
            doubleArrayPublishers.clear();
            doubleArrayEntries.clear();
            stringPublishers.clear();
            stringEntries.clear();

            // make a new log file?
            if (log_file != null)
                log_file.close();
            log_file = new DataLog("", "", 0.1);
            System.out.println("impl " + log_file.getImpl());
            Instant i = Instant.ofEpochSecond(timestamp);
            // TODO: sometimes this fails because the internal
            // "impl" is somehow null!?
            log_file.setFilename("FRC_" + m_timeFormatter.format(i) + ".wpilog");

            // restart the NT server?
            if (inst != null)
                inst.close();
            inst = NetworkTableInstance.getDefault();
            inst.startServer();

            m_timestamp = timestamp;
            return true;
        }
        return true;
    }

    @Override
    public void acceptBoolean(int key, boolean val) {
        if (m_timestamp == 0)
            return;
        counter.incrementAndGet();
        System.out.println("bool " + val);
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
        if (m_timestamp == 0)
            return;
        counter.incrementAndGet();
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
        if (m_timestamp == 0)
            return;
        counter.incrementAndGet();
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
        if (m_timestamp == 0)
            return;
        counter.incrementAndGet();
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
        if (m_timestamp == 0)
            return;
        counter.incrementAndGet();
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
        counter.incrementAndGet();
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

    /**
     * Network Tables has a compile-time 2MB output buffer, so it would be good to
     * call flush() often enough to keep it from filling up (thus dropping values).
     */
    @Override
    public void flush() {
        if (PUB)
            inst.flush();
        if (LOG)
            log_file.flush();
    }

    @Override
    public void close() {
        scheduler.shutdown();
    }
}
