package org.team100.lib.logging.primitive;

import java.util.Arrays;

public enum UdpType {
    UNKNOWN(0),
    BOOLEAN(1),
    DOUBLE(2),
    INT(3),
    DOUBLE_ARRAY(4),
    LONG(5),
    STRING(6);

    public final byte id;

    private static final UdpType[] list = new UdpType[values().length];

    static {
        Arrays.fill(list, UNKNOWN);
        for (UdpType u : values()) {
            list[u.id] = u;
        }
    }

    public static UdpType get(byte id) {
        if (id < 0)
            return UNKNOWN;
        if (id > values().length)
            return UNKNOWN;
        return list[id];
    }

    private UdpType(int typeId) {
        id = (byte) typeId;
    }
}