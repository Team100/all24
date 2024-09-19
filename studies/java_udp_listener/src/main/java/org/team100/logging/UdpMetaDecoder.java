package org.team100.logging;

import java.nio.ByteBuffer;

import org.team100.lib.telemetry.UdpPrimitiveProtocol2;
import org.team100.lib.telemetry.UdpPrimitiveProtocol2.ProtocolException;
import org.team100.lib.telemetry.UdpType;

public class UdpMetaDecoder {
    private final UdpConsumersInterface m_consumers;

    public UdpMetaDecoder(UdpConsumersInterface consumers) {
        m_consumers = consumers;
    }

    /** Starts at buf.position() */
    public void decode(ByteBuffer buf) throws ProtocolException {
        int key = UdpPrimitiveProtocol2.decodeKey(buf);
        UdpType type = UdpPrimitiveProtocol2.decodeType(buf);
        String v = UdpPrimitiveProtocol2.decodeString(buf);
        m_consumers.acceptMeta(key, type, v);
    }
}
