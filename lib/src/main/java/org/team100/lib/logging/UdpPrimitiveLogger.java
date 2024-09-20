package org.team100.lib.logging;

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
public class UdpPrimitiveLogger extends PrimitiveLogger {
    private static final double kFlushPeriod = 0.1;
    /** how many handles to transmit per iteration */
    private static final double kHandlesPeriodic = 10;
    private static final int kPort = 1995;

    /** nullable */
    private final InetAddress m_addr;
    /** nullable */
    private final DatagramSocket m_socket;
    // hang on to the buffer to prevent GC churn
    private final byte[] m_bytes;
    private final ByteBuffer m_bb;
    // lots of queues to avoid encoding values we're not going to send
    private final Map<String, Boolean> booleanQueue = new HashMap<>();
    private final Map<String, Double> doubleQueue = new HashMap<>();
    private final Map<String, Integer> integerQueue = new HashMap<>();
    private final Map<String, double[]> doubleArrayQueue = new HashMap<>();
    private final Map<String, Long> longQueue = new HashMap<>();
    private final Map<String, String> stringQueue = new HashMap<>();

    // this eventually goes into two bytes, so not the full int size
    // but casting to short everywhere is a pain
    final Map<String, Integer> handles = new HashMap<>();
    private Iterator<Map.Entry<String, Integer>> handleIterator;

    private double flushTime;

    public UdpPrimitiveLogger() {
        m_addr = makeAddr();
        m_socket = makeSocket();
        // 1k seems like enough!
        m_bytes = new byte[1000];
        m_bb = ByteBuffer.wrap(m_bytes);
        // this is the default, but just to make it clear...
        m_bb.order(ByteOrder.BIG_ENDIAN);
        flushTime = 0;
        handles.put("UNKNOWN", 0);
    }

    private int getHandle(String key) {
        return handles.computeIfAbsent(key, x -> handles.size());
    }

    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (flushTime + kFlushPeriod < now) {
            flush();
            flushTime = now;
        }
        if (handleIterator == null)
            handleIterator = handles.entrySet().iterator();
        for (int i =0; i < kHandlesPeriodic; ++i) {
            if (handleIterator.hasNext()) {
                Map.Entry<String, Integer> entry = handleIterator.next();
                String key = entry.getKey();
                Integer handle = entry.getValue();
                // TODO: better encoding, separate key space
                logString(handle.toString(), key);
            } else {
                handleIterator = null;
                break;
            }
        }
    }

    /** Send queued packets */
    public void flush() {
        flushBoolean();
        flushDouble();
        flushInteger();
        flushDoubleArray();
        flushLong();
        flushString();
    }

    @Override
    void logBoolean(String key, boolean val) {
        int handle = getHandle(key);
        // TODO: use the handle instead of the key here
        booleanQueue.put(key, val);
    }

    @Override
    void logDouble(String key, double val) {
        int handle = getHandle(key);
        // TODO: use the handle instead of the key here
        doubleQueue.put(key, val);
    }

    @Override
    void logInt(String key, int val) {
        int handle = getHandle(key);
        // TODO: use the handle instead of the key here
        integerQueue.put(key, val);
    }

    @Override
    void logDoubleArray(String key, double[] val) {
        int handle = getHandle(key);
        // TODO: use the handle instead of the key here
        doubleArrayQueue.put(key, val);
    }

    @Override
    void logDoubleObjArray(String key, Double[] val) {
        logDoubleArray(key, Stream.of(val).mapToDouble(Double::doubleValue).toArray());
    }

    @Override
    void logLong(String key, long val) {
        int handle = getHandle(key);
        // TODO: use the handle instead of the key here
        longQueue.put(key, val);
    }

    @Override
    void logString(String key, String val) {
        int handle = getHandle(key);
        // TODO: use the handle instead of the key here
        stringQueue.put(key, val);
    }

    ///////////////////////////////////////////

    private static InetAddress makeAddr() {
        try {
            // 10.1.0.16 is the one that i happen to have
            // TODO: make a dedicated log listener at 10.1.0.100.
            // return InetAddress.getByAddress(new byte[] { 10, 1, 0, 16 });
            return InetAddress.getLocalHost();
        } catch (UnknownHostException e) {
            e.printStackTrace();
            return null;
        }
    }

    private static DatagramSocket makeSocket() {
        try {
            DatagramSocket datagramSocket = new DatagramSocket();
            // big buffer does not help but doesn't hurt
            datagramSocket.setSendBufferSize(10000000);
            return datagramSocket;
        } catch (SocketException e) {
            e.printStackTrace();
            return null;
        }
    }

    private void flushBoolean() {
        Iterator<Map.Entry<String, Boolean>> it = booleanQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, Boolean> entry = it.next();
            int len = UdpPrimitiveProtocol.encodeBoolean(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void flushDouble() {
        Iterator<Map.Entry<String, Double>> it = doubleQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, Double> entry = it.next();
            int len = UdpPrimitiveProtocol.encodeDouble(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void flushInteger() {
        Iterator<Map.Entry<String, Integer>> it = integerQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, Integer> entry = it.next();
            int len = UdpPrimitiveProtocol.encodeInt(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void flushDoubleArray() {
        Iterator<Map.Entry<String, double[]>> it = doubleArrayQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, double[]> entry = it.next();
            int len = UdpPrimitiveProtocol.encodeDoubleArray(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void flushLong() {
        Iterator<Map.Entry<String, Long>> it = longQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, Long> entry = it.next();
            int len = UdpPrimitiveProtocol.encodeLong(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void flushString() {
        Iterator<Map.Entry<String, String>> it = stringQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, String> entry = it.next();
            int len = UdpPrimitiveProtocol.encodeString(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void send(DatagramPacket p) {
        if (m_socket == null)
            return;
        try {
            m_socket.send(p);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
