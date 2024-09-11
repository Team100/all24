package org.team100.lib.telemetry;

import java.util.List;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;

import org.team100.lib.util.Util;

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
 * for that type.
 * 
 * The payload can be either a chunk of labels, or a data payload.
 * 
 * A chunk of labels is an 2-byte offset followed by a string array.
 * 
 * Data payloads are lists of key-value pairs, repeating until a type code
 * appears instead of a key.
 * 
 * Data value format varies: generally it includes a length and some data.
 * Primitive arrays must have a length parameter since there is no way to
 * reserve a terminating value, so we use the same length-then-data pattern
 * everywhere. It makes decoding a bit easier too.
 * 
 * The maximum packet length is 508 bytes, to maximize delivery, which means
 * that lists are never very long; list lengths can be one byte.
 * 
 * Here's an example packet:
 * 
 * <pre>
 * TTNOOLAAAALAAAAATTLKKIIIIKKIIIITTLKKBTT
 * ^^                                      type = 7 (label)
 *   ^^                                    key of first label = 16
 *     ^                                   number of labels = 2
 *      ^                                  string length = 4
 *       ^^^^                              string in ascii
 *           ^                             string length = 5
 *            ^^^^^                        string in ascii
 *                 ^^                      type = 3 (int)
 *                   ^                     number of ints = 2
 *                    ^^                   key = 16 (the first label above)
 *                      ^^^^               int value = 1234 (4 bytes)
 *                          ^^             key = 17 (the second label above)
 *                            ^^^^         int value = 5678 (4 bytes)
 *                                ^^       type = 1 (boolean)
 *                                  ^      number of bools = 1
 *                                   ^^    key = 18 (an unseen label)
 *                                     ^   boolean value = true (1 byte)
 *                                      ^^ type = 0 (end of packet)
 * </pre>
 */
public class UdpPrimitiveProtocol2 {
    private enum Types {
        BOOLEAN(1),
        DOUBLE(2),
        INT(3),
        DOUBLE_ARRAY(4),
        LONG(5),
        STRING(6),
        LABEL(7);

        public final byte id;

        private Types(int typeId) {
            id = (byte) typeId;
        }
    }

    /**
     * <pre>
     * LKKKKKKKKKK
     *  ^^^^^^^^^^ ascii-encoded key
     * ^           key string length
     * total length = key length + 1
     * </pre>
     */
    private static int encodeKey(ByteBuffer buf, String key) {
        byte[] strBytes = key.getBytes(StandardCharsets.US_ASCII);
        int strLen = strBytes.length;
        if (strLen > 255) {
            Util.warn("key too long: " + key);
            strLen = 255;
        }
        buf.put((byte) strLen); // 1
        buf.put(strBytes, 0, strLen); // strLen
        return strLen + 1;
    }

    /**
     * An int key is written as 2 bytes, unsigned.
     */
    static int encodeKey(ByteBuffer buf, int key) {
        buf.putChar((char) key);
        return 2;
    }

    /**
     * <pre>
     * Bb
     * ^  boolean type id
     *  ^ boolean value
     * </pre>
     */
    static int encodeBoolean(ByteBuffer buf, String key, boolean val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.BOOLEAN.id); // 1
        buf.put(val ? (byte) 1 : (byte) 0); // 1 for bool
        return keyFieldLen + 2;
    }

    /**
     * <pre>
     * Ddddddddd
     * ^         double type id
     *  ^^^^^^^^ double value
     * </pre>
     */
    static int encodeDouble(ByteBuffer buf, String key, double val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.DOUBLE.id); // 1
        buf.putDouble(val); // 8 for double
        return keyFieldLen + 9;
    }

    /**
     * <pre>
     * Iiiii
     * ^     int type id
     *  ^^^^ int value
     * </pre>
     */
    static int encodeInt(ByteBuffer buf, String key, int val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.INT.id); // 1
        buf.putInt(val); // 4 for int
        return keyFieldLen + 5;
    }

    /**
     * <pre>
     * Dlllldddddddddddddddd
     * ^                     double array type id
     *  ^^^^                 array length
     *      ^^^^^^^^         double value 0
     *              ^^^^^^^^ double value 1
     * </pre>
     */
    static int encodeDoubleArray(ByteBuffer buf, String key, double[] val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.DOUBLE_ARRAY.id); // 1
        int length = val.length;
        if (length > 100) {
            length = 100;
            Util.warn("array too long for key " + key);
        }
        buf.putInt(length); // 4 for int
        for (int i = 0; i < length; ++i) {
            buf.putDouble(val[i]); // 8 for double
        }
        return keyFieldLen + length * 8 + 5;
    }

    /**
     * <pre>
     * Lllllllll
     * ^         long type id
     *  ^^^^^^^^ long value
     * </pre>
     */
    static int encodeLong(ByteBuffer buf, String key, long val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, key);
        buf.put(Types.LONG.id); // 1
        buf.putLong(val); // 8 for long
        return keyFieldLen + 9;
    }

    /**
     * <pre>
     * Sllllssssssssssssss
     * ^                   string type id
     *  ^^^^               string length
     *      ^^^^^^^^^^^^^^ string value
     * </pre>
     */
    static int encodeString(ByteBuffer buf, int key, String val) {
        int keyFieldLen = encodeKey(buf, key);
        byte[] bytes = val.getBytes(StandardCharsets.US_ASCII);
        if (bytes.length > 255)
            throw new IllegalArgumentException();
        buf.putInt(bytes.length); // 4 for int
        buf.put(bytes);
        return keyFieldLen + bytes.length + 4;
    }

    /**
     * encode a chunk of the label map.
     */
    static int encodeLabelMap(ByteBuffer buf, int offset, List<String> val) {
        buf.rewind();
        int keyFieldLen = encodeKey(buf, "foo");
        buf.put(Types.STRING.id); // 1
        buf.putInt(offset); // 4 for int
        buf.putInt(val.size()); // 4 for int
        for (String s : val) {
            buf.putInt(s.length());
            buf.put(s.getBytes(StandardCharsets.US_ASCII));
        }

        return keyFieldLen + 0;
    }
}
