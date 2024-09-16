package org.team100.lib.telemetry;

enum UdpType {
    UNKNOWN(0),
    BOOLEAN(1),
    DOUBLE(2),
    INT(3),
    DOUBLE_ARRAY(4),
    LONG(5),
    STRING(6);

    public final byte id;

    private UdpType(int typeId) {
        id = (byte) typeId;
    }
}