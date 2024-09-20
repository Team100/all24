package org.team100.lib.logging;

import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;

import org.team100.lib.util.Util;

/**
 * The log protocol is very simple:
 * 
 * One UDP packet per log entry.
 * 
 * First byte is key length (max 255)
 * Next N bytes are ASCII-encoded key
 * Next byte is value type, 1 to 6
 * Next N bytes are the ByteBuffer-encoded value.
 * 
 * Double array type is slightly different: instead of a single ByteBuffer-encoded value there is:
 *
 * Four bytes (int) indicating the length of the array
 * Next N bytes are values from the array, 8 bytes each
 * 
 * String is also different:
 * Four bytes (int) indicating the length of the string
 * Next N bytes are ASCII-encoded string.
 * 
 * TODO: replace string key with a numeric handle
 * 
 * TODO: add support for network tables timestamp alignment.
 */
public class UdpPrimitiveProtocol {
    private enum Types {
        BOOLEAN(1),
        DOUBLE(2),
        INT(3),
        DOUBLE_ARRAY(4),
        LONG(5),
        STRING(6);

        public final byte id;

        private Types(int typeId) {
            id = (byte) typeId;
        }
    }

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
