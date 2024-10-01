package org.team100.lib.logging.primitive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

class UdpMetadataProtocolTest {

    @Test
    void testLabelMap() {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(2);
        assertTrue(UdpMetadataProtocol.add(bb, 16, UdpType.INT, "one"));
        assertTrue(UdpMetadataProtocol.add(bb, 17, UdpType.INT, "two"));
        assertFalse(UdpMetadataProtocol.add(bb, 18, UdpType.INT, "three")); // no room
        assertFalse(UdpMetadataProtocol.add(bb, 19, UdpType.INT, "four"));
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key
        assertEquals((byte) 16, b[3]); // key
        assertEquals((byte) 3, b[4]); // type
        assertEquals((byte) 3, b[5]); // length
        assertEquals((byte) 111, b[6]); // o
        assertEquals((byte) 110, b[7]); // n
        assertEquals((byte) 101, b[8]); // e
        assertEquals((byte) 0, b[9]); // key
        assertEquals((byte) 17, b[10]); // key
        assertEquals((byte) 3, b[11]); // type
        assertEquals((byte) 3, b[12]); // length
        assertEquals((byte) 116, b[13]); // t
        assertEquals((byte) 119, b[14]); // w
        assertEquals((byte) 111, b[15]); // o
    }
}
