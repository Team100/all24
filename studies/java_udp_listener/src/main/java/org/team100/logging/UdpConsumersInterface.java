package org.team100.logging;

import org.team100.lib.telemetry.UdpType;

public interface UdpConsumersInterface {

    void acceptBoolean(int key, boolean val);

    void acceptDouble(int key, double val);

    void acceptInt(int key, int val);

    void acceptDoubleArray(int key, double[] val);

    void acceptString(int key, String val);

    void acceptMeta(int key, UdpType type, String val);

    void flush();

}