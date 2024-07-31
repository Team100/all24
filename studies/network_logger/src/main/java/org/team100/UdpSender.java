package org.team100;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;

public class UdpSender {
    private enum Types {
        BOOLEAN(1),
        DOUBLE(2),
        INT(3),
        FLOAT(4),
        DOUBLE_ARRAY(5),
        LONG(6),
        STRING(7);

        public final byte id;

        private Types(int typeId) {
            id = (byte) typeId;
        }
    }

    private static final int kPort = 1995;
    private final InetAddress m_addr;
    private final DatagramSocket m_socket;
    // hang on to the buffer to prevent GC churn
    private final byte[] m_bytes;
    private final ByteBuffer m_bb;

    public UdpSender() throws SocketException, UnknownHostException {
        // m_addr = InetAddress.getByAddress(new byte[] { 10, 1, 0, 100 });
        m_addr = InetAddress.getLocalHost();
        m_socket = new DatagramSocket();
        // 1k seems like enough!
        m_bytes = new byte[1000];
        m_bb = ByteBuffer.wrap(m_bytes);
        // this is the default, but just to make it clear...
        m_bb.order(ByteOrder.BIG_ENDIAN);
    }

    public void sendBoolean(String key, boolean val) throws IOException {
        int len = encodeBoolean(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        m_socket.send(p);
    }

    public void sendDouble(String key, double val) throws IOException {
        int len = encodeDouble(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        m_socket.send(p);
    }

    public void sendInt(String key, int val) throws IOException {
        int len = encodeInt(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        m_socket.send(p);
    }

    public void sendFloat(String key, float val) throws IOException {
        int len = encodeFloat(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        m_socket.send(p);
    }

    public void sendDoubleArray(String key, double[] val) throws IOException {
        int len = encodeDoubleArray(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        m_socket.send(p);
    }

    public void sendLong(String key, long val) throws IOException {
        int len = encodeLong(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        m_socket.send(p);
    }

    public void sendString(String key, String val) throws IOException {
        int len = encodeString(m_bb, key, val);
        DatagramPacket p = new DatagramPacket(m_bytes, len, m_addr, kPort);
        m_socket.send(p);
    }

    public void close() {
        m_socket.close();
    }

    static int encodeKey(ByteBuffer buf, String key) {
        byte[] strBytes = key.getBytes(StandardCharsets.US_ASCII);
        int strLen = strBytes.length;
        buf.put((byte) strLen); // 1
        buf.put(strBytes); // strLen
        return strLen + 1;
    }

    static int encodeBoolean(ByteBuffer buf, String key, boolean val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.BOOLEAN.id); // 1
        buf.put(val ? (byte) 1 : (byte) 0); // 1 for bool
        return keyFieldLen + 2;
    }

    static int encodeDouble(ByteBuffer buf, String key, double val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.DOUBLE.id); // 1
        buf.putDouble(val); // 8 for double
        return keyFieldLen + 9;
    }

    static int encodeInt(ByteBuffer buf, String key, int val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.INT.id); // 1
        buf.putInt(val); // 4 for int
        return keyFieldLen + 5;
    }

    static int encodeFloat(ByteBuffer buf, String key, float val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.FLOAT.id); // 1
        buf.putFloat(val); // 4 for float
        return keyFieldLen + 5;
    }

    static int encodeDoubleArray(ByteBuffer buf, String key, double[] val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.DOUBLE_ARRAY.id); // 1
        buf.putInt(val.length); // 4 for int
        for (double v : val) {
            buf.putDouble(v); // 8 for double
        }
        return keyFieldLen + val.length * 8 + 5;
    }

    static int encodeLong(ByteBuffer buf, String key, long val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.LONG.id); // 1
        buf.putLong(val); // 8 for long
        return keyFieldLen + 9;
    }

    static int encodeString(ByteBuffer buf, String key, String val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.STRING.id); // 1
        byte[] bytes = val.getBytes(StandardCharsets.US_ASCII);
        buf.putInt(bytes.length); // 4 for int
        buf.put(bytes);
        return keyFieldLen + bytes.length + 5;
    }
}
