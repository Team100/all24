package org.team100.lib.telemetry;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.stream.Stream;

/**
 * Send logs to a log recipient via UDP.
 * 
 * UDP is not formally reliable but on the robot LAN, for log data, it's good
 * enough.
 * 
 * The recipient IP is always 10.1.0.100. (or 16 at the moment)
 */
public class UdpPrimitiveLogger extends PrimitiveLogger {
    private static final int kPort = 1995;

    /** nullable */
    private final InetAddress m_addr;
    /** nullable */
    private final DatagramSocket m_socket;
    // hang on to the buffer to prevent GC churn
    private final byte[] m_bytes;
    private final ByteBuffer m_bb;

    private static InetAddress makeAddr() {
        try {
            // 10.1.0.16 is the one that i happen to have
            // TODO: make a dedicated log listener at 10.1.0.100.
            return InetAddress.getByAddress(new byte[] { 10, 1, 0, 16 });
            // return InetAddress.getLocalHost();
        } catch (UnknownHostException e) {
            e.printStackTrace();
            return null;
        }
    }

    private static DatagramSocket makeSocket() {
        try {
            return new DatagramSocket();
        } catch (SocketException e) {
            e.printStackTrace();
            return null;
        }
    }

    public UdpPrimitiveLogger() {
        m_addr = makeAddr();
        m_socket = makeSocket();
        // 1k seems like enough!
        m_bytes = new byte[1000];
        m_bb = ByteBuffer.wrap(m_bytes);
        // this is the default, but just to make it clear...
        m_bb.order(ByteOrder.BIG_ENDIAN);
    }

    private void send(DatagramPacket p) {
        if (m_socket == null)
            return;
        try {
            m_socket.send(p);
        } catch (IOException e) {
        }
    }

    @Override
    void logBoolean(String key, boolean val) {
        int len = UdpPrimitiveProtocol.encodeBoolean(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        send(p);
    }

    @Override
    void logDouble(String key, double val) {
        int len = UdpPrimitiveProtocol.encodeDouble(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        send(p);
    }

    @Override
    void logInt(String key, int val) {
        int len = UdpPrimitiveProtocol.encodeInt(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        send(p);
    }

    @Override
    void logDoubleArray(String key, double[] val) {
        int len = UdpPrimitiveProtocol.encodeDoubleArray(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        send(p);
    }

    @Override
    void logDoubleObjArray(String key, Double[] val) {
        logDoubleArray(key, Stream.of(val).mapToDouble(Double::doubleValue).toArray());
    }

    @Override
    void logLong(String key, long val) {
        int len = UdpPrimitiveProtocol.encodeLong(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        send(p);
    }

    @Override
    void logString(String key, String val) {
        int len = UdpPrimitiveProtocol.encodeString(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        send(p);
    }

}
