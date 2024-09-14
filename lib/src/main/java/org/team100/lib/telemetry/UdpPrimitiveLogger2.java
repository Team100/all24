package org.team100.lib.telemetry;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;
import java.util.stream.Stream;
import java.util.function.Consumer;
import java.util.function.BooleanSupplier;
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
    private static final int kMinKey = 16;
    private static final double kFlushPeriod = 0.1;
    /** how many handles to transmit per iteration */
    private static final double kHandlesPeriodic = 10;

    // lots of queues to avoid encoding values we're not going to send
    private final Map<Integer, Boolean> booleanQueue = new HashMap<>();
    private final Map<Integer, Double> doubleQueue = new HashMap<>();
    private final Map<Integer, Integer> integerQueue = new HashMap<>();
    private final Map<Integer, double[]> doubleArrayQueue = new HashMap<>();
    private final Map<Integer, Long> longQueue = new HashMap<>();
    private final Map<Integer, String> stringQueue = new HashMap<>();

    final List<String> handles = new ArrayList<>();
    private Iterator<Map.Entry<String, Integer>> handleIterator;

    private double flushTime;

    // since the protocol is now stateful, representing the "current message,"
    // we keep it here.
    private UdpPrimitiveProtocol2 p;

    private Consumer<ByteBuffer> m_bufferSink;

    public UdpPrimitiveLogger2(Consumer<ByteBuffer> bufferSink) {
        m_bufferSink = bufferSink;

        flushTime = 0;
        // handles.put("UNKNOWN", 0);
    }

    /**
     * Call this once when the specific logger class is instantiated.
     */
    private synchronized int getKey(String label) {
        int key = handles.size() + kMinKey;
        handles.add(label);
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

    public void dumpLabels() {
        // if (handleIterator == null)
        // handleIterator = handles.entrySet().iterator();
        for (int i = 0; i < kHandlesPeriodic; ++i) {
            if (handleIterator.hasNext()) {
                Map.Entry<String, Integer> entry = handleIterator.next();
                String key = entry.getKey();
                Integer handle = entry.getValue();
                // TODO: better encoding, separate key space
                // logString(handle.toString(), key);
            } else {
                handleIterator = null;
                break;
            }
        }
    }

    /** Send at least one packet. */
    public void flush() {
        p = new UdpPrimitiveProtocol2();
        // TODO: flush some key map now

        flushBoolean();
        flushDouble();
        flushInteger();
        flushDoubleArray();
        flushLong();
        flushString();
        ByteBuffer trim = p.trim();
        System.out.println(trim.array().length);
        m_bufferSink.accept(trim);
    }

    public class UdpBooleanLogger implements PrimitiveLogger2.BooleanLogger {
        private final int m_key;

        public UdpBooleanLogger(int key) {
            m_key = key;
        }

        @Override
        public void log(boolean val) {
            booleanQueue.put(m_key, val);
        }
    }

    public class UdpDoubleLogger implements PrimitiveLogger2.DoubleLogger {
        private final int m_key;

        public UdpDoubleLogger(String label) {
            m_key = getKey(label);
        }

        @Override
        public void log(double val) {
            doubleQueue.put(m_key, val);
        }

    }

    public class UdpIntLogger implements PrimitiveLogger2.IntLogger {
        private final int m_key;

        public UdpIntLogger(String label) {
            m_key = getKey(label);
        }

        @Override
        public void log(int val) {
            integerQueue.put(m_key, val);
        }
    }

    public class UdpDoubleArrayLogger implements PrimitiveLogger2.DoubleArrayLogger {
        private final int m_key;

        public UdpDoubleArrayLogger(String label) {
            m_key = getKey(label);
        }

        @Override
        public void log(double[] val) {
            doubleArrayQueue.put(m_key, val);
        }
    }

    public class UdpDoubleObjArrayLogger implements PrimitiveLogger2.DoubleObjArrayLogger {
        private final int m_key;

        public UdpDoubleObjArrayLogger(String label) {
            m_key = getKey(label);
        }

        @Override
        public void log(Double[] val) {
            doubleArrayQueue.put(m_key, Stream.of(val).mapToDouble(Double::doubleValue).toArray());
        }
    }

    public class UdpLongLogger implements PrimitiveLogger2.LongLogger {
        private final int m_key;

        public UdpLongLogger(String label) {
            m_key = getKey(label);
        }

        @Override
        public void log(long val) {
            longQueue.put(m_key, val);
        }

    }

    public class UdpStringLogger implements PrimitiveLogger2.StringLogger {
        private final int m_key;

        public UdpStringLogger(String label) {
            m_key = getKey(label);
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
}
