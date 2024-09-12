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
import java.util.stream.Stream;
import java.util.function.Consumer;

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
public class UdpPrimitiveLogger2 extends PrimitiveLogger {
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
    private final Map<Integer, Boolean> booleanQueue = new HashMap<>();
    private final Map<Integer, Double> doubleQueue = new HashMap<>();
    private final Map<Integer, Integer> integerQueue = new HashMap<>();
    private final Map<Integer, double[]> doubleArrayQueue = new HashMap<>();
    private final Map<Integer, Long> longQueue = new HashMap<>();
    private final Map<Integer, String> stringQueue = new HashMap<>();

    // this eventually goes into two bytes, so not the full int size
    // but casting to short everywhere is a pain
    final Map<String, Integer> handles = new HashMap<>();
    private Iterator<Map.Entry<String, Integer>> handleIterator;

    private double flushTime;

    // since the protocol is now stateful, representing the "current message,"
    // we keep it here.
    private UdpPrimitiveProtocol2 p;

    private Consumer<DatagramPacket> m_packetSink;

    public UdpPrimitiveLogger2(Consumer<DatagramPacket> packetSink) {
        m_packetSink = packetSink;
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

    // TODO: this should only be used once when the logger instance is made for the
    // specific label, to avoid the map lookups.
    private int getKey(String label) {
        return handles.computeIfAbsent(label, x -> handles.size());
    }

    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (flushTime + kFlushPeriod < now) {
            flush();
            flushTime = now;
        }
        if (handleIterator == null)
            handleIterator = handles.entrySet().iterator();
        for (int i = 0; i < kHandlesPeriodic; ++i) {
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

    /** Send at least one packet. */
    public void flush() {
        p = new UdpPrimitiveProtocol2();
        flushBoolean();
        flushDouble();
        flushInteger();
        flushDoubleArray();
        flushLong();
        flushString();
    }

    @Override
    void logBoolean(String label, boolean val) {
        int key = getKey(label);
        booleanQueue.put(key, val);
    }

    @Override
    void logDouble(String label, double val) {
        int key = getKey(label);
        doubleQueue.put(key, val);
    }

    @Override
    void logInt(String label, int val) {
        int key = getKey(label);
        integerQueue.put(key, val);
    }

    @Override
    void logDoubleArray(String label, double[] val) {
        int key = getKey(label);
        doubleArrayQueue.put(key, val);
    }

    @Override
    void logDoubleObjArray(String label, Double[] val) {
        logDoubleArray(label, Stream.of(val).mapToDouble(Double::doubleValue).toArray());
    }

    @Override
    void logLong(String label, long val) {
        int key = getKey(label);
        longQueue.put(key, val);
    }

    @Override
    void logString(String label, String val) {
        int key = getKey(label);
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
        Iterator<Map.Entry<Integer, Boolean>> it = booleanQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Integer, Boolean> entry = it.next();
            boolean written = p.putBoolean(entry.getKey(), entry.getValue());
            if (!written) {
                // time to send the packet
                send(new DatagramPacket(m_bytes, p.buffer().position(), m_addr, kPort));
            }
            int len = UdpPrimitiveProtocol2.encodeBoolean(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void flushDouble() {
        Iterator<Map.Entry<Integer, Double>> it = doubleQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Integer, Double> entry = it.next();
            int len = UdpPrimitiveProtocol2.encodeDouble(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void flushInteger() {
        Iterator<Map.Entry<Integer, Integer>> it = integerQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Integer, Integer> entry = it.next();
            int len = UdpPrimitiveProtocol2.encodeInt(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void flushDoubleArray() {
        Iterator<Map.Entry<Integer, double[]>> it = doubleArrayQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Integer, double[]> entry = it.next();
            int len = UdpPrimitiveProtocol2.encodeDoubleArray(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void flushLong() {
        Iterator<Map.Entry<Integer, Long>> it = longQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Integer, Long> entry = it.next();
            int len = UdpPrimitiveProtocol2.encodeLong(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void flushString() {
        Iterator<Map.Entry<Integer, String>> it = stringQueue.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Integer, String> entry = it.next();
            int len = UdpPrimitiveProtocol2.encodeString(m_bb, entry.getKey(), entry.getValue());
            send(new DatagramPacket(m_bytes, len, m_addr, kPort));
            it.remove();
        }
    }

    private void send(DatagramPacket p) {
        if (m_socket == null)
            return;
        try {
            m_packetSink.accept(p);
            // m_socket.send(p);
        } catch (IOException e) {
            e.printStackTrace();
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
