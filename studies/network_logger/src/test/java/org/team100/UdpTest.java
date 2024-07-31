package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.util.HexFormat;

import org.junit.jupiter.api.Test;

class UdpTest {
    @Test
    void testSimple() throws InterruptedException, IOException {
        for (int i = 0; i < 100; ++i) {
            String message = "hello";
            byte[] buf = message.getBytes();
            InetAddress addr = InetAddress.getLocalHost();
            DatagramPacket p = new DatagramPacket(buf, buf.length, addr, 1995);
            DatagramSocket socket = new DatagramSocket();
            socket.send(p);
            System.out.println("sent");
            Thread.sleep(100);
        }
    }

    @Test
    void testSender() throws IOException {
        UdpSender sender = new UdpSender();
        for (int i = 0; i < 100; ++i) {
            sender.sendBoolean("key", true);
        }
    }

    @Test
    void testEncodeBoolean() {
        HexFormat hex = HexFormat.of();
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        int len = UdpSender.encodeBoolean(bb, "key", true);
        assertEquals(6, len);
        assertEquals("03" // key.length
                + "6b" + "65" + "79" // "key"
                + "01" // indicator for bool
                + "01" // true
                + "00000000", //
                hex.formatHex(b));

    }

    @Test
    void testEncodeDouble() {
        HexFormat hex = HexFormat.of();
        byte[] b = new byte[15];
        ByteBuffer bb = ByteBuffer.wrap(b);
        int len = UdpSender.encodeDouble(bb, "key", Math.PI);
        assertEquals(13, len);
        assertEquals("03" // key.length
                + "6b" + "65" + "79" // "key"
                + "02" // indicator for double
                + "400921fb54442d18" // double
                + "0000", //
                hex.formatHex(b));
    }

    @Test
    void testEncodeInt() {
        HexFormat hex = HexFormat.of();
        byte[] b = new byte[15];
        ByteBuffer bb = ByteBuffer.wrap(b);
        int len = UdpSender.encodeInt(bb, "key", 100);
        assertEquals(9, len);
        assertEquals("03" // key.length
                + "6b" + "65" + "79" // "key"
                + "03" // indicator for int
                + "00000064" // int
                + "000000000000", //
                hex.formatHex(b));
    }

    @Test
    void testEncodeFloat() {
        HexFormat hex = HexFormat.of();
        byte[] b = new byte[15];
        ByteBuffer bb = ByteBuffer.wrap(b);
        int len = UdpSender.encodeFloat(bb, "key", (float)100.0);
        assertEquals(9, len);
        assertEquals("03" // key.length
                + "6b" + "65" + "79" // "key"
                + "04" // indicator for float
                + "42c80000" // float
                + "000000000000", //
                hex.formatHex(b));
    }
}
