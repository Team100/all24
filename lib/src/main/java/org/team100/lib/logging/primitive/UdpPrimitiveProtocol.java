package org.team100.lib.logging.primitive;

import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;

/**
 * Log data protocol 2
 * 
 * Data packets are lists of tuples:
 * 
 * * key (2 bytes)
 * * type (1 byte)
 * * value (varies)
 * 
 * I previously had a more complicated, terse, stateful protocol, but I think
 * it's worth a few bytes to be simpler.
 * 
 * This protocol is very simple: input tuples, write them to a buffer.
 * 
 * The type is included here so that the parser knows how to parse the value.
 * 
 * <pre>
 * DDDDDDDDKKTIIIIKKTIIIIKKTB
 * ^^^^^^^^                   timestamp
 *         ^^                 key = 16
 *           ^                type = 5 (int)
 *            ^^^^            int value = 1234 (4 bytes)
 *                ^^          key = 17
 *                  ^         type = 5 (int)
 *                   ^^^^     int value = 5678 (4 bytes)
 *                       ^^   key = 18
 *                         ^  type = 3 (bool)
 *                          ^ boolean value = true (1 byte)
 * </pre>
 * 
 * Decoders throw ProtocolException in case of bounds exceptions.
 * 
 * These operations used to use offsets but the only use cases are
 * sequential, so i converted them all to relative.
 */
public class UdpPrimitiveProtocol {
    public static class ProtocolException extends Exception {
    }

    private final ByteBuffer m_buffer;

    public UdpPrimitiveProtocol(int bufferSize) {
        // direct buffer goes slightly faster out the network
        m_buffer = ByteBuffer.allocateDirect(bufferSize);
        // big-endian is the default, but just to make it clear...
        m_buffer.order(ByteOrder.BIG_ENDIAN);
        m_buffer.putLong(UdpMetadataProtocol.timestamp); // timetstamp = 8 bytes
    }

    public UdpPrimitiveProtocol() {
        this(UdpSender.MTU);
    }

    /** Return a buffer view of length equal to current position. */
    ByteBuffer trim() {
        return m_buffer.slice(0, m_buffer.position());
    }

    /** for testing */
    ByteBuffer buffer() {
        return m_buffer;
    }

    /** Clear the underlying buffer and rewrite the timestamp into it. */
    void clear() {
        m_buffer.clear();
        m_buffer.putLong(UdpMetadataProtocol.timestamp);
    }

    /** @return true if written */
    public boolean putLong(int key, long val) {
        return encodeLong(m_buffer, key, val) != 0;
    }

    /** @return true if written */
    public boolean putString(int key, String val) {
        return encodeString(m_buffer, key, val) != 0;
    }

    /** @return true if written */
    public boolean putInt(int key, int val) {
        return encodeInt(m_buffer, key, val) != 0;
    }

    /** @return true if written */
    public boolean putDouble(int key, double val) {
        return encodeDouble(m_buffer, key, val) != 0;
    }

    /** @return true if written */
    public boolean putBoolean(int key, boolean val) {
        return encodeBoolean(m_buffer, key, val) != 0;
    }

    /** @return true if written */
    public boolean putDoubleArray(int key, double[] val) {
        return encodeDoubleArray(m_buffer, key, val) != 0;
    }

    /** just the key */
    public static int decodeKey(ByteBuffer buf) throws ProtocolException {
        try {
            return buf.getChar();
        } catch (IndexOutOfBoundsException | BufferUnderflowException e) {
            throw new ProtocolException();
        }
    }

    /** just the type */
    public static UdpType decodeType(ByteBuffer buf) throws ProtocolException {
        try {
            return UdpType.get(buf.get());
        } catch (IndexOutOfBoundsException | BufferUnderflowException e) {
            throw new ProtocolException();
        }
    }

    // TODO: use bool as return value for the encoders; i never use the offset
    // number anyway.

    /**
     * <pre>
     * KKTb
     * ^^   key (2 bytes)
     *   ^  type (1 byte)
     *    ^ boolean value (1 byte)
     * </pre>
     */
    static int encodeBoolean(ByteBuffer buf, int key, boolean val) {
        final int totalLength = 4;
        if (buf.remaining() < totalLength)
            return 0;
        buf.putChar((char) key); // 2 bytes
        buf.put(UdpType.BOOLEAN.id); // type = 1 byte
        buf.put(val ? (byte) 1 : (byte) 0); // 1 byte
        return totalLength;
    }

    public static boolean decodeBoolean(ByteBuffer buf) throws ProtocolException {
        try {
            return buf.get() != 0;
        } catch (IndexOutOfBoundsException | BufferUnderflowException e) {
            throw new ProtocolException();
        }
    }

    /**
     * <pre>
     * KKTdddddddd
     * ^^          key (2 bytes)
     *   ^         type (1 byte)
     *    ^^^^^^^^ double value (8 bytes)
     * </pre>
     */
    static int encodeDouble(ByteBuffer buf, int key, double val) {
        final int totalLength = 11;
        if (buf.remaining() < totalLength)
            return 0;
        buf.putChar((char) key); // 2 bytes
        buf.put(UdpType.DOUBLE.id); // type = 1 byte
        buf.putDouble(val); // 8 bytes
        return totalLength;
    }

    /** just the double part */
    public static double decodeDouble(ByteBuffer buf) throws ProtocolException {
        try {
            return buf.getDouble();
        } catch (IndexOutOfBoundsException | BufferUnderflowException e) {
            throw new ProtocolException();
        }
    }

    /**
     * <pre>
     * KKTiiii
     * ^^      key (2 bytes)
     *   ^     type (1 byte)
     *    ^^^^ int value (4 bytes)
     * </pre>
     */
    static int encodeInt(ByteBuffer buf, int key, int val) {
        final int totalLength = 7;
        if (buf.remaining() < totalLength)
            return 0;
        buf.putChar((char) key); // 2 bytes
        buf.put(UdpType.INT.id); // type = 1 byte
        buf.putInt(val); // 4 bytes
        return totalLength;
    }

    public static int decodeInt(ByteBuffer buf) throws ProtocolException {
        try {
            return buf.getInt();
        } catch (IndexOutOfBoundsException | BufferUnderflowException e) {
            throw new ProtocolException();
        }
    }

    /**
     * Note the maximum array length is not very long (approximately packet length
     * divided by 8).
     * 
     * <pre>
     * KKTldddddddddddddddd
     * ^^                   key (2 bytes)
     *   ^                  type (1 byte)
     *    ^                 array length (1 byte)
     *     ^^^^^^^^         double value 0
     *             ^^^^^^^^ double value 1
     * </pre>
     */
    static int encodeDoubleArray(ByteBuffer buf, int key, double[] val) {
        if (val.length > 255)
            throw new IllegalArgumentException();
        final int totalLength = 4 + val.length * 8;
        if (buf.remaining() < totalLength)
            return 0;
        buf.putChar((char) key); // 2 bytes
        buf.put(UdpType.DOUBLE_ARRAY.id); // type = 1 byte
        buf.put((byte) val.length); // 1 byte
        for (int i = 0; i < val.length; ++i) {
            buf.putDouble(val[i]); // 8 bytes
        }
        return totalLength;
    }

    public static double[] decodeDoubleArray(ByteBuffer buf) throws ProtocolException {
        try {
            int length = buf.get();
            double[] result = new double[length];
            for (int i = 0; i < length; ++i) {
                result[i] = buf.getDouble();
            }
            return result;
        } catch (IndexOutOfBoundsException | BufferUnderflowException e) {
            throw new ProtocolException();
        }
    }

    /**
     * Note: try to avoid logging long ints, they're needlessly ... long.
     * 
     * <pre>
     * KKTllllllll
     * ^^          key (2 bytes)
     *   ^         type (1 byte)
     *    ^^^^^^^^ long value (8 bytes)
     * </pre>
     */
    static int encodeLong(ByteBuffer buf, int key, long val) {
        final int totalLength = 11;
        if (buf.remaining() < totalLength)
            return 0;
        buf.putChar((char) key); // 2 bytes
        buf.put(UdpType.LONG.id); // type = 1 byte
        buf.putLong(val); // 8 bytes
        return totalLength;
    }

    public static long decodeLong(ByteBuffer buf) throws ProtocolException {
        try {
            return buf.getLong();
        } catch (IndexOutOfBoundsException | BufferUnderflowException e) {
            throw new ProtocolException();
        }
    }

    /**
     * Note: try to avoid logging strings.
     * 
     * <pre>
     * KKLTssssssssssss
     * ^^               key (2 bytes)
     *   ^              type (1 byte)
     *    ^             string length (1 byte)
     *     ^^^^^^^^^^^^ string value (255 bytes max)
     * </pre>
     */
    static int encodeString(ByteBuffer buf, int key, String val) {
        final byte[] bytes = val.getBytes(StandardCharsets.US_ASCII);
        final int bytesLength = bytes.length;
        if (bytesLength > 255)
            throw new IllegalArgumentException();
        final int totalLength = 4 + bytesLength;
        if (buf.remaining() < totalLength)
            return 0;
        buf.putChar((char) key); // 2 bytes
        buf.put(UdpType.STRING.id); // type = 1 byte
        buf.put((byte) bytesLength); // 1 byte
        buf.put(bytes);
        return totalLength;
    }

    public static String decodeString(ByteBuffer buf) throws ProtocolException {
        try {
            int length = buf.get();
            byte[] bytes = new byte[length];
            buf.get(bytes);
            return new String(bytes, StandardCharsets.US_ASCII);
        } catch (IndexOutOfBoundsException | BufferUnderflowException e) {
            throw new ProtocolException();
        }
    }
}
