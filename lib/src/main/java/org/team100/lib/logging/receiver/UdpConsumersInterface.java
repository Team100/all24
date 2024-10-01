package org.team100.lib.logging.receiver;

import org.team100.lib.logging.primitive.UdpType;

public interface UdpConsumersInterface {

    boolean validateTimestamp(long timestamp);

    void acceptBoolean(int key, boolean val);

    void acceptDouble(int key, double val);

    void acceptInt(int key, int val);

    void acceptDoubleArray(int key, double[] val);

    void acceptString(int key, String val);

    void acceptMeta(int key, UdpType type, String val);

    void flush();

    void close();
}