package org.team100.lib.telemetry;

import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;

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
