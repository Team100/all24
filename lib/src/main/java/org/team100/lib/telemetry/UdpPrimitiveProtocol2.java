package org.team100.lib.telemetry;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * Log protocol 2
 * 
 * The main design criteria for this format are speed and simplicity, for the
 * lossy, approximate-time logging use case. There's no timestamping, there are
 * only a few types, just what is needed for live and offline log analysis.
 * 
 * The sender maintains a map of string keys to (2-byte) integer handles (these
 * are actually 16-bit java "char" type), and sends fragments of this map to the
 * receiver periodically, so that the receiver can maintain an
 * eventually-consistent copy of the key map. Handles are 2 bytes to allow more
 * than 256 keys, which is a common case, but less than 65535, way more than
 * we'll ever have. Note there is no virtue in using the "short" type, because
 * it's signed, slower than int, and the same in RAM (Java uses 32-bit
 * boundaries).
 * 
 * The key map is dense (i.e. it's just a list), so it is transmitted in dense
 * chunks (i.e. lists).
 * 
 * There's no guarantee about the size or distribution or frequency of map
 * updates, other than eventually every row will be sent within any long-enough
 * interval. The client should ignore key handles it doesn't recognize.
 * 
 * Packets are sent via UDP, which means they can be dropped; experimentally,
 * about 1% of packets are dropped. Do not use this method for data that must
 * not be dropped.
 * 
 * Each packet is a list of sections.
 * 
 * Each section starts with a two-byte type code, followed by a list of payloads
 * for that type. Why two bytes? Because the type code is substituted for a data
 * key to indicate a change in type, and data keys are 2 bytes. The minimum real
 * key is 16, to leave some space for new type codes.
 * 
 * The payload can be either a chunk of labels, or a data payload.
 * 
 * A chunk of labels is an 2-byte offset followed by a string array. There is no
 * length field, because that would require laying out all the labels to see if
 * they fit.
 * 
 * A data payload consists of key-value pairs. Sometimes the key is a type code,
 * which indicates the start of a new section.
 * 
 * The value format varies. For fixed-length array type (double[], string),
 * there is
 * a length field, because the length can be calculated easily.
 * 
 * The maximum packet length is 508 bytes, to maximize delivery, which means
 * that lists are never very long; list lengths can be one byte.
 * 
 * Here's an example packet:
 * 
 * <pre>
 * TTOOLAAAALAAAAATTKKIIIIKKIIIITTKKBTT
 * ^^                                   type = 7 (label)
 *   ^^                                 key of first label = 16
 *     ^                                string length = 4
 *      ^^^^                            string in ascii
 *          ^                           string length = 5
 *           ^^^^^                      string in ascii
 *                ^^                    type = 3 (int)
 *                  ^^                  key = 16 (the first label above)
 *                    ^^^^              int value = 1234 (4 bytes)
 *                        ^^            key = 17 (the second label above)
 *                          ^^^^        int value = 5678 (4 bytes)
 *                              ^^      type = 1 (boolean)
 *                                ^^    key = 18 (an unseen label)
 *                                  ^   boolean value = true (1 byte)
 *                                   ^^ type = 0 (end of packet)
 * </pre>
 * 
 * Note this is a stateful protocol within the scope of a single message,
 * because of the "current type". This implies that the message itself is
 * part of the state, so it is contained here. And because it would be
 * confusing to reuse, each instance is intended to be used once.
 * 
 * <pre>
 * new message -> set type X -> values of type X -> set type Y -> ...
 * </pre>
 * 
 * It is an error to mix value types "set type" in between, so the API here
 * makes it impossible to do so, inserting "set type" as needed. callers
 * should keep same-type fields together to minimize these extra bytes.
 */
public class UdpPrimitiveProtocol2 {
    enum Type {
        BOOLEAN(1),
        DOUBLE(2),
        INT(3),
        DOUBLE_ARRAY(4),
        LONG(5),
        STRING(6),
        LABEL(7);

        public final byte id;

        private Type(int typeId) {
            id = (byte) typeId;
        }
    }

    private Type m_currentType;
    private final ByteBuffer m_buffer;

    public UdpPrimitiveProtocol2() {
        m_buffer = ByteBuffer.allocate(508);
        // this is the default, but just to make it clear...
        m_buffer.order(ByteOrder.BIG_ENDIAN);
    }

    /** Return a buffer view of length equal to current position. */
    ByteBuffer trim() {
        ByteBuffer slice = m_buffer.slice(0, m_buffer.position());
        System.out.println(slice.array().length);
        return slice;
    }

    /** for testing */
    ByteBuffer buffer() {
        return m_buffer;
    }

    /** @return true if the type is correct or a new type was written */
    private boolean verifyType(Type type) {
        if (m_currentType != type) {
            int l = encodeType(m_buffer, type);
            if (l == 0)
                return false;
        }
        return true;
    }

    /**
     * @return true if written
     */
    public boolean putLong(int key, long val) {
        if (verifyType(Type.LONG))
            return encodeLong(m_buffer, key, val) != 0;
        return false;
    }

    public boolean putString(int key, String val) {
        if (verifyType(Type.STRING))
            return encodeString(m_buffer, key, val) != 0;
        return false;
    }

    public boolean putInt(int key, int val) {
        if (verifyType(Type.INT))
            return encodeInt(m_buffer, key, val) != 0;
        return false;
    }

    public boolean putDouble(int key, double val) {
        if (verifyType(Type.DOUBLE))
            return encodeDouble(m_buffer, key, val) != 0;
        return false;
    }

    public boolean putBoolean(int key, boolean val) {
        if (verifyType(Type.BOOLEAN))
            return encodeBoolean(m_buffer, key, val) != 0;
        return false;
    }

    public boolean putDoubleArray(int key, double[] val) {
        if (verifyType(Type.DOUBLE_ARRAY))
            return encodeDoubleArray(m_buffer, key, val) != 0;
        return false;
    }

    /**
     * @param offset start with this item
     * @param length number of items to try to send
     * @param val    the entire label list; this is not modified
     * @return the number of items actually sent
     */
    public int putLabels(int offset, int length, List<String> val) {
        if (verifyType(Type.LABEL))
            return encodeLabels(m_buffer, offset, length, val);
        return 0;
    }

    /**
     * Encode a type
     * 
     * <pre>
     * TT
     * ^^ value (2 bytes)
     * </pre>
     */
    static int encodeType(ByteBuffer buf, Type val) {
        final int totalLength = 2;
        if (buf.remaining() < totalLength)
            return 0;
        buf.putChar((char) val.id); // 2 bytes
        return totalLength;
    }

    /**
     * encode a chunk of the label map.
     * 
     * the label map is long, we have to chunk it.
     * 
     * the caller gives the entire list, and we report how many items were sent (not
     * the bytes sent).
     * 
     * Note the "offset" here is not the same as the actual encoded label id.
     *
     * <pre>
     * OOSLAAALAAA
     * ^^          offset
     *   ^         number of labels actually sent
     *    ^        length
     *     ^^^     string
     *        ^    length
     *         ^^^ string
     * </pre>
     * 
     * @param buf    write into this buffer
     * @param offset start with this item (0-based)
     * @param length number of items to try to send
     * @param val    the entire label list; this is not modified
     * @return the number of items actually sent
     * 
     */
    static int encodeLabels(ByteBuffer buf, int offset, int length, List<String> val) {
        if (offset > (val.size() - 1))
            throw new IllegalArgumentException("label offset too large");
        if (length > (val.size() - offset))
            throw new IllegalArgumentException("length too long");
        if (val.size() > 255) {
            throw new IllegalArgumentException("label list too long");
        }

        // how many items can we fit?
        final int bufRemaining = buf.remaining();
        int totalLength = 3;
        List<byte[]> bytesList = new ArrayList<>();
        for (int i = offset; i < offset + length; ++i) {
            String s = val.get(i);
            byte[] bytes = s.getBytes(StandardCharsets.US_ASCII);
            final int bytesLength = bytes.length;
            if (bytesLength > 255)
                throw new IllegalArgumentException("label too long");
            totalLength += bytesLength + 1;
            if (totalLength > bufRemaining) // this one won't fit
                break;
            bytesList.add(bytes);
        }

        if (bytesList.isEmpty())
            return 0;

        buf.putChar((char) offset); // 2 bytes
        buf.put((byte) bytesList.size()); // 1 byte
        for (byte[] bytes : bytesList) {
            buf.put((byte) bytes.length); // 1 byte
            buf.put(bytes);
        }

        return bytesList.size();
    }

    /**
     * Note the maximum array length is not very long (approximately packet length
     * divided by 8).
     * 
     * <pre>
     * KKldddddddddddddddd
     * ^^                  key (2 bytes)
     *   ^                 array length (1 byte)
     *    ^^^^^^^^         double value 0
     *            ^^^^^^^^ double value 1
     * </pre>
     */
    static int encodeDoubleArray(ByteBuffer buf, int key, double[] val) {
        if (val.length > 255) {
            throw new IllegalArgumentException();
        }
        final int keyFieldLen = encodeKey(buf, key);
        final int totalLength = keyFieldLen + val.length * 8 + 1;
        if (buf.remaining() < totalLength)
            return 0;
        buf.put((byte) val.length); // 1 byte
        for (int i = 0; i < val.length; ++i) {
            buf.putDouble(val[i]); // 8 bytes
        }
        return totalLength;
    }

    /**
     * <pre>
     * KKb
     * ^^  key (2 bytes)
     *   ^ boolean value (1 byte)
     * </pre>
     */
    static int encodeBoolean(ByteBuffer buf, int key, boolean val) {
        final int keyFieldLen = encodeKey(buf, key);
        final int totalLength = keyFieldLen + 1;
        if (buf.remaining() < totalLength)
            return 0;
        buf.put(val ? (byte) 1 : (byte) 0); // 1 byte
        return totalLength;
    }

    /**
     * <pre>
     * KKdddddddd
     * ^^         key (2 bytes)
     *   ^^^^^^^^ double value (8 bytes)
     * </pre>
     */
    static int encodeDouble(ByteBuffer buf, int key, double val) {
        final int keyFieldLen = encodeKey(buf, key);
        final int totalLength = keyFieldLen + 8;
        if (buf.remaining() < totalLength)
            return 0;
        buf.putDouble(val); // 8 bytes
        return totalLength;
    }

    /**
     * <pre>
     * KKiiii
     * ^^     key (2 bytes)
     *   ^^^^ int value (4 bytes)
     * </pre>
     */
    static int encodeInt(ByteBuffer buf, int key, int val) {
        final int keyFieldLen = encodeKey(buf, key);
        final int totalLength = keyFieldLen + 4;
        if (buf.remaining() < totalLength)
            return 0;
        buf.putInt(val); // 4 bytes
        return totalLength;
    }

    /**
     * Note: try to avoid logging strings.
     * 
     * <pre>
     * KKLSSSSSSSSSSSS
     * ^^              key (2 bytes)
     *   ^             string length (1 byte)
     *    ^^^^^^^^^^^^ string value (255 bytes max)
     * </pre>
     */
    static int encodeString(ByteBuffer buf, int key, String val) {
        final byte[] bytes = val.getBytes(StandardCharsets.US_ASCII);
        final int bytesLength = bytes.length;
        if (bytesLength > 255)
            throw new IllegalArgumentException();
        final int keyFieldLen = encodeKey(buf, key);
        final int totalLength = keyFieldLen + bytesLength + 1;
        if (buf.remaining() < totalLength)
            return 0;
        buf.put((byte) bytesLength); // 1 byte
        buf.put(bytes);
        return totalLength;
    }

    /**
     * Note: try to avoid logging long ints, they're needlessly ... long.
     * 
     * <pre>
     * KKllllllll
     * ^^         key (2 bytes)
     *   ^^^^^^^^ long value (8 bytes)
     * </pre>
     */
    static int encodeLong(ByteBuffer buf, int key, long val) {
        final int keyFieldLen = encodeKey(buf, key);
        final int totalLength = keyFieldLen + 8;
        if (buf.remaining() < totalLength)
            return 0;
        buf.putLong(val); // 8 bytes
        return totalLength;
    }

    /**
     * An int key is written as 2 bytes, unsigned.
     * 
     * @return bytes written. zero means we didn't write anything, buffer is full.
     */
    static int encodeKey(ByteBuffer buf, int key) {
        if (key < 16)
            throw new IllegalArgumentException("key in type code range: " + key);
        if (key > 65535)
            throw new IllegalArgumentException("key too large");
        final int totalLength = 2;
        if (buf.remaining() < totalLength)
            return 0;
        buf.putChar((char) key); // 2 bytes
        return totalLength;
    }

}
