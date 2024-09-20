package org.team100.lib.logging.receiver;

import java.nio.ByteBuffer;

import org.team100.lib.logging.UdpPrimitiveProtocol2;
import org.team100.lib.logging.UdpType;
import org.team100.lib.logging.UdpPrimitiveProtocol2.ProtocolException;

public class UdpDataDecoder {
    private static final int kFlushFrequency = 50;
    private final UdpConsumersInterface m_consumers;
    private int flushCounter = 0;

    public UdpDataDecoder(UdpConsumersInterface consumers) {
        m_consumers = consumers;
    }

    /**
     * return true if timestamp is the first we've seen, or the same as the previous
     * one.
     * @throws ProtocolException 
     */
    public boolean validateTimestamp(ByteBuffer buf) throws ProtocolException {
        long timestamp = UdpPrimitiveProtocol2.decodeLong(buf);
        return m_consumers.validateTimestamp(timestamp);
    }

    /**
     * Starts at buf.position()
     * Flushes the consumers at the end.
     */
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
        if (flushCounter++ > kFlushFrequency) {
            m_consumers.flush();
            flushCounter = 0;
        }
    }
}
