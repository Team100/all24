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
    void testSimple() throws IOException {
        InetAddress addr = InetAddress.getLocalHost();
        DatagramSocket socket = new DatagramSocket();
        try {
            for (int i = 0; i < 100; ++i) {
                String message = "hello";
                byte[] buf = message.getBytes();
                DatagramPacket p = new DatagramPacket(buf, buf.length, addr, 1995);
                socket.send(p);
                System.out.println("sent");
            }
        } finally {
            socket.close();
        }
    }

    @Test
    void testSender() throws IOException {
        UdpSender sender = new UdpSender();
        try {
            for (int i = 0; i < 100; ++i) {
                sender.sendBoolean("key", true);
                sender.sendDouble("key", 100.0);
                sender.sendInt("key", (int) 100);
                sender.sendFloat("key", (float)100);
                sender.sendDoubleArray("key", new double[]{1.0, 2.0});
                sender.sendLong("key", (long)100);
                sender.sendString("key", "value");
            }
        } finally {
            sender.close();
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
        int len = UdpSender.encodeDouble(bb, "key", 100.0);
        assertEquals(13, len);
        assertEquals("03" // key.length
                + "6b" + "65" + "79" // "key"
                + "02" // indicator for double
                + "4059000000000000" // double
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
        int len = UdpSender.encodeFloat(bb, "key", (float) 100.0);
        assertEquals(9, len);
        assertEquals("03" // key.length
                + "6b" + "65" + "79" // "key"
                + "04" // indicator for float
                + "42c80000" // float
                + "000000000000", //
                hex.formatHex(b));
    }

    @Test
    void testEncodeDoubleArray() {
        HexFormat hex = HexFormat.of();
        byte[] b = new byte[30];
        ByteBuffer bb = ByteBuffer.wrap(b);
        int len = UdpSender.encodeDoubleArray(bb, "key", new double[] { 1.0, 2.0 });
        assertEquals(25, len);
        assertEquals("03" // key.length
                + "6b" + "65" + "79" // "key"
                + "05" // indicator for double array
                + "00000002" // array length 2
                + "3ff0000000000000" // 1.0
                + "4000000000000000" // 2.0
                + "0000000000", //
                hex.formatHex(b));
    }

    @Test
    void testEncodeLong() {
        HexFormat hex = HexFormat.of();
        byte[] b = new byte[15];
        ByteBuffer bb = ByteBuffer.wrap(b);
        int len = UdpSender.encodeLong(bb, "key", (long) 100);
        assertEquals(13, len);
        assertEquals("03" // key.length
                + "6b" + "65" + "79" // "key"
                + "06" // indicator for long
                + "0000000000000064" // long
                + "0000", //
                hex.formatHex(b));
    }

    @Test
    void testEncodeString() {
        HexFormat hex = HexFormat.of();
        byte[] b = new byte[15];
        ByteBuffer bb = ByteBuffer.wrap(b);
        int len = UdpSender.encodeString(bb, "key", "value");
        assertEquals(14, len);
        assertEquals("03" // key.length
                + "6b" + "65" + "79" // "key"
                + "07" // indicator for string
                + "00000005" // value length
                + "76" + "61" + "6c" + "75" + "65" // "value"
                + "00", //
                hex.formatHex(b));
    }
}
