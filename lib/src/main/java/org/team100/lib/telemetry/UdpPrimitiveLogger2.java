package org.team100.lib.telemetry;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
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
    }

    private static final double kFlushPeriod = 0.1;

    // lots of queues to avoid encoding values we're not going to send
    private final Map<Integer, Boolean> booleanQueue = new HashMap<>();
    private final Map<Integer, Double> doubleQueue = new HashMap<>();
    private final Map<Integer, Integer> integerQueue = new HashMap<>();
    private final Map<Integer, double[]> doubleArrayQueue = new HashMap<>();
    private final Map<Integer, Long> longQueue = new HashMap<>();
    private final Map<Integer, String> stringQueue = new HashMap<>();

    final List<Metadata> metadata = new ArrayList<>();
    /** Current offset of label dumper */
    int offset = 0;

    private double flushTime;

    private UdpPrimitiveProtocol2 p;
    private UdpMetadataProtocol m;

    private Consumer<ByteBuffer> m_bufferSink;
    private Consumer<ByteBuffer> m_metadataSink;

    public UdpPrimitiveLogger2(Consumer<ByteBuffer> bufferSink, Consumer<ByteBuffer> metadataSink) {
        m_bufferSink = bufferSink;
        m_metadataSink = metadataSink;
        flushTime = 0;
    }

    /**
     * Call this once when the specific logger class is instantiated.
     */
    private synchronized int getKey(UdpType type, String label) {
        int key = metadata.size();
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
        m = new UdpMetadataProtocol();
        for (int i = offset; i < metadata.size(); ++i) {
            Metadata d = metadata.get(i);
            if (!m.put(d.key, d.type, d.label)) {
                // packet is full, so send it.
                m_metadataSink.accept(m.trim());
                offset = i;
                return true;
            }
        }
        // added them all, send what we have.
        m_metadataSink.accept(m.trim());
        offset = 0;
        return false;
    }

    /** Send at least one packet. */
    public void flush() {
        p = new UdpPrimitiveProtocol2();
        flushBoolean();
        flushDouble();
        flushInteger();
        flushDoubleArray();
        flushLong();
        flushString();
        m_bufferSink.accept(p.trim());
    }

    public class UdpBooleanLogger implements PrimitiveLogger2.BooleanLogger {
        private final int m_key;

        public UdpBooleanLogger(String label) {
            m_key = getKey(UdpType.BOOLEAN, label);
        }

        @Override
        public void log(boolean val) {
            booleanQueue.put(m_key, val);
        }
    }

    public class UdpDoubleLogger implements PrimitiveLogger2.DoubleLogger {
        private final int m_key;

        public UdpDoubleLogger(String label) {
            m_key = getKey(UdpType.DOUBLE, label);
        }

        @Override
        public void log(double val) {
            doubleQueue.put(m_key, val);
        }

    }

    public class UdpIntLogger implements PrimitiveLogger2.IntLogger {
        private final int m_key;

        public UdpIntLogger(String label) {
            m_key = getKey(UdpType.INT, label);
        }

        @Override
        public void log(int val) {
            integerQueue.put(m_key, val);
        }
    }

    public class UdpDoubleArrayLogger implements PrimitiveLogger2.DoubleArrayLogger {
        private final int m_key;

        public UdpDoubleArrayLogger(String label) {
            m_key = getKey(UdpType.DOUBLE_ARRAY, label);
        }

        @Override
        public void log(double[] val) {
            doubleArrayQueue.put(m_key, val);
        }
    }

    public class UdpDoubleObjArrayLogger implements PrimitiveLogger2.DoubleObjArrayLogger {
        private final int m_key;

        public UdpDoubleObjArrayLogger(String label) {
            m_key = getKey(UdpType.DOUBLE_ARRAY, label);
        }

        @Override
        public void log(Double[] val) {
            doubleArrayQueue.put(m_key, Stream.of(val).mapToDouble(Double::doubleValue).toArray());
        }
    }

    public class UdpLongLogger implements PrimitiveLogger2.LongLogger {
        private final int m_key;

        public UdpLongLogger(String label) {
            m_key = getKey(UdpType.LONG, label);
        }

        @Override
        public void log(long val) {
            longQueue.put(m_key, val);
        }

    }

    public class UdpStringLogger implements PrimitiveLogger2.StringLogger {
        private final int m_key;

        public UdpStringLogger(String label) {
            m_key = getKey(UdpType.STRING, label);
        }

        @Override
        public void log(String val) {
            stringQueue.put(m_key, val);
        }
    }

    ///////////////////////////////////////////

    /** @param putter puts the value if there's room, returns false if not. */
    private void putAndMaybeSend(BooleanSupplier putter) {
        if (!putter.getAsBoolean()) {
            // time to send the packet
            m_bufferSink.accept(p.trim());
            p = new UdpPrimitiveProtocol2();
            if (!putter.getAsBoolean())
                throw new IllegalStateException();
        }

    }

    private void flushBoolean() {
        final Iterator<Map.Entry<Integer, Boolean>> it = booleanQueue.entrySet().iterator();
        while (it.hasNext()) {
            final Map.Entry<Integer, Boolean> entry = it.next();
            putAndMaybeSend(() -> p.putBoolean(entry.getKey(), entry.getValue()));
            it.remove();
        }
    }

    private void flushDouble() {
        final Iterator<Map.Entry<Integer, Double>> it = doubleQueue.entrySet().iterator();
        while (it.hasNext()) {
            final Map.Entry<Integer, Double> entry = it.next();
            putAndMaybeSend(() -> p.putDouble(entry.getKey(), entry.getValue()));
            it.remove();
        }
    }

    private void flushInteger() {
        final Iterator<Map.Entry<Integer, Integer>> it = integerQueue.entrySet().iterator();
        while (it.hasNext()) {
            final Map.Entry<Integer, Integer> entry = it.next();
            putAndMaybeSend(() -> p.putInt(entry.getKey(), entry.getValue()));
            it.remove();
        }
    }

    private void flushDoubleArray() {
        final Iterator<Map.Entry<Integer, double[]>> it = doubleArrayQueue.entrySet().iterator();
        while (it.hasNext()) {
            final Map.Entry<Integer, double[]> entry = it.next();
            putAndMaybeSend(() -> p.putDoubleArray(entry.getKey(), entry.getValue()));
            it.remove();
        }
    }

    private void flushLong() {
        final Iterator<Map.Entry<Integer, Long>> it = longQueue.entrySet().iterator();
        while (it.hasNext()) {
            final Map.Entry<Integer, Long> entry = it.next();
            putAndMaybeSend(() -> p.putLong(entry.getKey(), entry.getValue()));
            it.remove();
        }
    }

    private void flushString() {
        final Iterator<Map.Entry<Integer, String>> it = stringQueue.entrySet().iterator();
        while (it.hasNext()) {
            final Map.Entry<Integer, String> entry = it.next();
            putAndMaybeSend(() -> p.putString(entry.getKey(), entry.getValue()));
            it.remove();
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
