package org.team100;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import org.junit.jupiter.api.Test;

class UdpTest {
    @Test
    void testSimple() throws InterruptedException, IOException {
        while (true) {
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
}
