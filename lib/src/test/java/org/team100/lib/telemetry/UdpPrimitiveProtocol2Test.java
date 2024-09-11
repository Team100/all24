package org.team100.lib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.junit.jupiter.api.Test;

class UdpPrimitiveProtocol2Test {
    @Test
    void testKey() {
        byte[] b = new byte[12];
        assertEquals((byte) 0, b[0]);
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.order(ByteOrder.BIG_ENDIAN);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeKey(bb, 16);
        assertEquals(2, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // high byte
        assertEquals((byte) 16, b[3]); // low byte
        assertEquals((byte) 0, b[4]);
    }

    @Test
    void testString() {
        byte[] b = new byte[12];
        assertEquals((byte) 0, b[0]);
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeString(bb, 16, "hello");
        assertEquals(2, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 0, b[4]); // length
        assertEquals((byte) 0, b[5]); // h
        assertEquals((byte) 0, b[6]); // e
        assertEquals((byte) 0, b[7]); // l
        assertEquals((byte) 0, b[8]); // l
        assertEquals((byte) 0, b[9]); // o
        assertEquals((byte) 0, b[10]);
        assertEquals((byte) 0, b[11]);
    }

}
