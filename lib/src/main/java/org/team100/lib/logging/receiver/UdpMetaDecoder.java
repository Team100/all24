package org.team100.lib.logging.receiver;

import java.nio.ByteBuffer;

import org.team100.lib.logging.UdpPrimitiveProtocol2;
import org.team100.lib.logging.UdpType;
import org.team100.lib.logging.UdpPrimitiveProtocol2.ProtocolException;

public class UdpMetaDecoder {
    private static final int kFlushFrequency = 50;

    private final UdpConsumersInterface m_consumers;
    private int flushCounter = 0;

    public UdpMetaDecoder(UdpConsumersInterface consumers) {
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

    /** Starts at buf.position() */
    public void decode(ByteBuffer buf) throws ProtocolException {
        int key = UdpPrimitiveProtocol2.decodeKey(buf);
        UdpType type = UdpPrimitiveProtocol2.decodeType(buf);
        String v = UdpPrimitiveProtocol2.decodeString(buf);
        m_consumers.acceptMeta(key, type, v);
        if (flushCounter++ > kFlushFrequency) {
            m_consumers.flush();
            flushCounter = 0;
        }
    }
}
