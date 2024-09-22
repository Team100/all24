package org.team100.lib.logging;

import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.Timer;

/**
 * Send logs to a log recipient via UDP.
 * 
 * UDP is not formally reliable but on the robot LAN, for log data, it's good
 * enough.
 * 
 * The recipient IP is always 10.1.0.100. (or 16 at the moment)
 * 
 * This logger accepts inputs only one value per key per flush period; the
 * newest value wins.
 */
public class UdpPrimitiveLogger2 implements PrimitiveLogger2 {
    record Metadata(int key, UdpType type, String label) {
        /** this is the only place we check the sizes. */
        public Metadata {
            if (key > 65535)
                throw new IllegalArgumentException("too many keys");
            byte[] bytes = label.getBytes(StandardCharsets.US_ASCII);
            if (bytes.length > 255)
                throw new IllegalArgumentException("label too long: " + label);
        }
    }

    private static final double kFlushPeriod = 0.1;

    private final List<UdpBooleanLogger> booleanLoggers = new ArrayList<>();
    private final List<UdpDoubleLogger> doubleLoggers = new ArrayList<>();
    private final List<UdpIntLogger> integerLoggers = new ArrayList<>();
    private final List<UdpDoubleArrayLogger> doubleArrayLoggers = new ArrayList<>();
    private final List<UdpDoubleObjArrayLogger> doubleObjArrayLoggers = new ArrayList<>();
    private final List<UdpLongLogger> longLoggers = new ArrayList<>();
    private final List<UdpStringLogger> stringLoggers = new ArrayList<>();

    final List<Metadata> metadata = new ArrayList<>();
    private final Set<String> keys = new HashSet<>();
    private final Consumer<ByteBuffer> m_bufferSink;
    private final Consumer<ByteBuffer> m_metadataSink;

    // keep the output buffers forever because allocating it is slow.
    private final UdpPrimitiveProtocol2 m_dataProtocol;
    private final UdpMetadataProtocol m_metadataProtocol;

    /** Current offset of label dumper */
    int offset = 0;

    private double flushTime;

    public UdpPrimitiveLogger2(
            Consumer<ByteBuffer> dataSink,
            Consumer<ByteBuffer> metadataSink) {
        m_bufferSink = dataSink;
        m_metadataSink = metadataSink;
        m_dataProtocol = new UdpPrimitiveProtocol2();
        m_metadataProtocol = new UdpMetadataProtocol();
        flushTime = 0;
    }

    /**
     * Call this once when the specific logger class is instantiated.
     * 
     * Minimum key is 1, so that zero is an "invalid key".
     */
    private synchronized int getKey(UdpType type, String label) {
        int key = metadata.size() + 1;
        if (keys.contains(label))
            throw new IllegalArgumentException("duplicate key " + key);
        keys.add(label);
        metadata.add(new Metadata(key, type, label));
        return key;
    }

    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (flushTime + kFlushPeriod < now) {
            flush();
            dumpLabels();
            flushTime = now;
        }
    }

    public void sendAllLabels() {
        while (dumpLabels())
            ;
    }

    /**
     * Send one packet of labels.
     * 
     * return true if there are more labels to send
     */
    public boolean dumpLabels() {
        if (metadata.isEmpty())
            return false;
        m_metadataProtocol.clear();
        for (int i = offset; i < metadata.size(); ++i) {
            Metadata d = metadata.get(i);
            if (!m_metadataProtocol.put(d.key, d.type, d.label)) {
                // packet is full, so send it.
                m_metadataSink.accept(m_metadataProtocol.trim());
                offset = i;
                return true;
            }
        }
        // added them all, send what we have.
        m_metadataSink.accept(m_metadataProtocol.trim());
        offset = 0;
        return false;
    }

    /** Send at least one packet. */
    public void flush() {
        m_dataProtocol.clear();
        flushBoolean();
        flushDouble();
        flushInteger();
        flushDoubleArray();
        flushDoubleObjArray();
        flushLong();
        flushString();
        m_bufferSink.accept(m_dataProtocol.trim());
    }

    public class UdpBooleanLogger implements PrimitiveLogger2.BooleanLogger {
        private final int m_key;
        private boolean m_val;
        private boolean m_dirty;

        public UdpBooleanLogger(String label) {
            m_key = getKey(UdpType.BOOLEAN, label);
            booleanLoggers.add(this);
        }

        @Override
        public void log(boolean val) {
            m_val = val;
            m_dirty = true;
        }
    }

    public class UdpDoubleLogger implements PrimitiveLogger2.DoubleLogger {
        private final int m_key;
        private double m_val;
        private boolean m_dirty;

        public UdpDoubleLogger(String label) {
            m_key = getKey(UdpType.DOUBLE, label);
            doubleLoggers.add(this);
        }

        @Override
        public void log(double val) {
            m_val = val;
            m_dirty = true;
        }

    }

    public class UdpIntLogger implements PrimitiveLogger2.IntLogger {
        private final int m_key;
        private int m_val;
        private boolean m_dirty;

        public UdpIntLogger(String label) {
            m_key = getKey(UdpType.INT, label);
            integerLoggers.add(this);
        }

        @Override
        public void log(int val) {
            m_val = val;
            m_dirty = true;
        }
    }

    public class UdpDoubleArrayLogger implements PrimitiveLogger2.DoubleArrayLogger {
        private final int m_key;
        private double[] m_val;
        private boolean m_dirty;

        public UdpDoubleArrayLogger(String label) {
            m_key = getKey(UdpType.DOUBLE_ARRAY, label);
            doubleArrayLoggers.add(this);
        }

        @Override
        public void log(double[] val) {
            m_val = val;
            m_dirty = true;
        }
    }

    public class UdpDoubleObjArrayLogger implements PrimitiveLogger2.DoubleObjArrayLogger {
        private final int m_key;
        private double[] m_val;
        private boolean m_dirty;

        public UdpDoubleObjArrayLogger(String label) {
            m_key = getKey(UdpType.DOUBLE_ARRAY, label);
            doubleObjArrayLoggers.add(this);
        }

        @Override
        public void log(Double[] val) {
            m_val = Stream.of(val).mapToDouble(Double::doubleValue).toArray();
            m_dirty = true;
        }
    }

    public class UdpLongLogger implements PrimitiveLogger2.LongLogger {
        private final int m_key;
        private long m_val;
        private boolean m_dirty;

        public UdpLongLogger(String label) {
            m_key = getKey(UdpType.LONG, label);
            longLoggers.add(this);
        }

        @Override
        public void log(long val) {
            m_val = val;
            m_dirty = true;
        }

    }

    public class UdpStringLogger implements PrimitiveLogger2.StringLogger {
        private final int m_key;
        private String m_val;
        private boolean m_dirty;

        public UdpStringLogger(String label) {
            m_key = getKey(UdpType.STRING, label);
            stringLoggers.add(this);
        }

        @Override
        public void log(String val) {
            m_val = val;
            m_dirty = true;
        }
    }

    ///////////////////////////////////////////

    /** @param putter puts the value if there's room, returns false if not. */
    private void putAndMaybeSend(BooleanSupplier putter) {
        if (!putter.getAsBoolean()) {
            // time to send the packet
            m_bufferSink.accept(m_dataProtocol.trim());
            m_dataProtocol.clear();
            if (!putter.getAsBoolean())
                throw new IllegalStateException();
        }

    }

    private void flushBoolean() {
        for (UdpBooleanLogger logger : booleanLoggers) {
            if (logger.m_dirty) {
                putAndMaybeSend(() -> m_dataProtocol.putBoolean(logger.m_key, logger.m_val));
                logger.m_dirty = false;
            }
        }
    }

    private void flushDouble() {
        for (UdpDoubleLogger logger : doubleLoggers) {
            if (logger.m_dirty) {
                putAndMaybeSend(() -> m_dataProtocol.putDouble(logger.m_key, logger.m_val));
                logger.m_dirty = false;
            }
        }
    }

    private void flushInteger() {
        for (UdpIntLogger logger : integerLoggers) {
            if (logger.m_dirty) {
                putAndMaybeSend(() -> m_dataProtocol.putInt(logger.m_key, logger.m_val));
                logger.m_dirty = false;
            }
        }
    }

    private void flushDoubleArray() {
        for (UdpDoubleArrayLogger logger : doubleArrayLoggers) {
            if (logger.m_dirty) {
                putAndMaybeSend(() -> m_dataProtocol.putDoubleArray(logger.m_key, logger.m_val));
                logger.m_dirty = false;
            }
        }
    }

    private void flushDoubleObjArray() {
        for (UdpDoubleObjArrayLogger logger : doubleObjArrayLoggers) {
            if (logger.m_dirty) {
                putAndMaybeSend(() -> m_dataProtocol.putDoubleArray(logger.m_key, logger.m_val));
                logger.m_dirty = false;
            }
        }
    }

    private void flushLong() {
        for (UdpLongLogger logger : longLoggers) {
            if (logger.m_dirty) {
                putAndMaybeSend(() -> m_dataProtocol.putLong(logger.m_key, logger.m_val));
                logger.m_dirty = false;
            }
        }
    }

    private void flushString() {
        for (UdpStringLogger logger : stringLoggers) {
            if (logger.m_dirty) {
                putAndMaybeSend(() -> m_dataProtocol.putString(logger.m_key, logger.m_val));
                logger.m_dirty = false;
            }
        }
    }

    @Override
    public BooleanLogger booleanLogger(String label) {
        return new UdpBooleanLogger(label);
    }

    @Override
    public DoubleLogger doubleLogger(String label) {
        return new UdpDoubleLogger(label);
    }

    @Override
    public IntLogger intLogger(String label) {
        return new UdpIntLogger(label);
    }

    @Override
    public DoubleArrayLogger doubleArrayLogger(String label) {
        return new UdpDoubleArrayLogger(label);
    }

    @Override
    public DoubleObjArrayLogger doubleObjArrayLogger(String label) {
        return new UdpDoubleObjArrayLogger(label);
    }

    @Override
    public LongLogger longLogger(String label) {
        return new UdpLongLogger(label);
    }

    @Override
    public StringLogger stringLogger(String label) {
        return new UdpStringLogger(label);
    }

}
