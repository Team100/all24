package org.team100.logging;

import java.nio.ByteBuffer;

import org.team100.lib.telemetry.UdpPrimitiveProtocol2;
import org.team100.lib.telemetry.UdpPrimitiveProtocol2.ProtocolException;
import org.team100.lib.telemetry.UdpType;

public class UdpDataDecoder {
    private final UdpConsumersInterface m_consumers;

    
    

    public UdpDataDecoder(UdpConsumersInterface consumers) {
        m_consumers = consumers;
    }

    /** Starts at buf.position() */
    public void decode(ByteBuffer buf) throws ProtocolException {
        int key = UdpPrimitiveProtocol2.decodeKey(buf);
        UdpType type = UdpPrimitiveProtocol2.decodeType(buf);
        switch (type) {
            case BOOLEAN -> {
                boolean v = UdpPrimitiveProtocol2.decodeBoolean(buf);
                m_consumers.acceptBoolean(key, v);
            }
            case DOUBLE -> {
                double v = UdpPrimitiveProtocol2.decodeDouble(buf);
                m_consumers.acceptDouble(key, v);
            }
            case INT -> {
                int v = UdpPrimitiveProtocol2.decodeInt(buf);
                m_consumers.acceptInt(key, v);
            }
            case DOUBLE_ARRAY -> {
                double[] v = UdpPrimitiveProtocol2.decodeDoubleArray(buf);
                m_consumers.acceptDoubleArray(key, v);
            }
            case LONG -> {
                long v = UdpPrimitiveProtocol2.decodeLong(buf);
                m_consumers.acceptInt(key, (int) v);
            }
            case STRING -> {
                String v = UdpPrimitiveProtocol2.decodeString(buf);
                m_consumers.acceptString(key, v);
            }
            default -> System.out.println("unknown data decoder type");
        }
    }
}
