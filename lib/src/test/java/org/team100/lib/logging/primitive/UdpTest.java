package org.team100.lib.logging.primitive;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.StandardSocketOptions;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Timer;

/** Timing is on my desktop, real address (not localhost), no listener. */
class UdpTest {
    private static final int PACKET_BUF_SIZE = 1472;
    private static final byte[] SOCKET_ADDR = new byte[] { 10, 1, 0, 16 };
    private static final int PORT = 1995;
    private static final int SOCKET_BUF_SIZE = 10000000;
    private static final int N = 200000;

    private final InetAddress m_addr;

    public UdpTest() throws UnknownHostException {
        m_addr = InetAddress.getByAddress(SOCKET_ADDR);
        // m_addr = InetAddress.getLocalHost(); // for testing
    }

    /**
     * Socket.send
     * 
     * 12 us per packet
     */
    @Test
    void test1() throws IOException {
        DatagramSocket m_socket = new DatagramSocket();
        m_socket.setSendBufferSize(SOCKET_BUF_SIZE);
        byte[] m_bytes = new byte[PACKET_BUF_SIZE];
        double t0 = Timer.getFPGATimestamp();
        for (int i = 0; i < N; ++i) {
            try {
                DatagramPacket p = new DatagramPacket(m_bytes, m_bytes.length, m_addr, PORT);
                m_socket.send(p);
            } catch (IOException e) {
                // windows throws here, which shouldn't happen.
            }
        }
        double t1 = Timer.getFPGATimestamp();
        System.out.printf("1: duration sec %5.3f\n", (t1 - t0));
        System.out.printf("1: duration per row us %5.3f\n", 1000000 * (t1 - t0) / N);
        m_socket.close();
    }

    /**
     * Socket.connect and then socket.send.
     * 
     * 2.2 us per packet
     */
    @Test
    void test2() throws IOException {
        DatagramSocket m_socket = new DatagramSocket();
        m_socket.setSendBufferSize(SOCKET_BUF_SIZE);
        m_socket.connect(m_addr, PORT);
        byte[] m_bytes = new byte[30];
        double t0 = Timer.getFPGATimestamp();
        for (int i = 0; i < N; ++i) {
            try {
                DatagramPacket p = new DatagramPacket(m_bytes, m_bytes.length, m_addr, PORT);
                m_socket.send(p);
            } catch (IOException e) {
                // windows throws here, which shouldn't happen.
            }
        }
        double t1 = Timer.getFPGATimestamp();
        System.out.printf("2: duration sec %5.3f\n", (t1 - t0));
        System.out.printf("2: duration per row us %5.3f\n", 1000000 * (t1 - t0) / N);
        m_socket.close();
    }

    /**
     * Channel.connect and then channel.write (non-blocking)
     * 
     * 1.2 us per packet
     */
    @Test
    void test3() throws IOException {
        DatagramChannel channel = DatagramChannel.open();
        channel.configureBlocking(false);
        channel.setOption(StandardSocketOptions.SO_SNDBUF, SOCKET_BUF_SIZE);
        InetSocketAddress sockAddr = new InetSocketAddress(m_addr, PORT);
        channel.connect(sockAddr);
        byte[] m_bytes = new byte[PACKET_BUF_SIZE];
        ByteBuffer m_bb = ByteBuffer.wrap(m_bytes);
        double t0 = Timer.getFPGATimestamp();
        for (int i = 0; i < N; ++i) {
            m_bb.rewind();
            try {
                channel.write(m_bb);
            } catch (IOException e) {
                // windows throws here, which shouldn't happen.
            }
        }
        double t1 = Timer.getFPGATimestamp();
        System.out.printf("3: duration sec %5.3f\n", (t1 - t0));
        System.out.printf("3: duration per row us %5.3f\n", 1000000 * (t1 - t0) / N);
        channel.close();
    }

    /**
     * Channel.send (non-blocking, no connect)
     * 
     * 1.3 us per packet
     */
    @Test
    void test4() throws IOException {
        DatagramChannel channel = DatagramChannel.open();
        channel.configureBlocking(false);
        channel.setOption(StandardSocketOptions.SO_SNDBUF, SOCKET_BUF_SIZE);
        InetSocketAddress sockAddr = new InetSocketAddress(m_addr, PORT);
        byte[] m_bytes = new byte[PACKET_BUF_SIZE];
        ByteBuffer m_bb = ByteBuffer.wrap(m_bytes);
        double t0 = Timer.getFPGATimestamp();
        for (int i = 0; i < N; ++i) {
            m_bb.rewind();
            channel.send(m_bb, sockAddr);
        }
        double t1 = Timer.getFPGATimestamp();
        System.out.printf("4: duration sec %5.3f\n", (t1 - t0));
        System.out.printf("4: duration per row us %5.3f\n", 1000000 * (t1 - t0) / N);
        channel.close();
    }

    /**
     * Channel.send (non-blocking) with direct buffer, no connect
     * 
     * 1.2 us per packet
     */
    @Test
    void test5() throws IOException {
        DatagramChannel channel = DatagramChannel.open();
        channel.configureBlocking(false);
        channel.setOption(StandardSocketOptions.SO_SNDBUF, SOCKET_BUF_SIZE);
        InetSocketAddress sockAddr = new InetSocketAddress(m_addr, PORT);
        ByteBuffer m_bb = ByteBuffer.allocateDirect(PACKET_BUF_SIZE);
        double t0 = Timer.getFPGATimestamp();
        for (int i = 0; i < N; ++i) {
            m_bb.rewind();
            channel.send(m_bb, sockAddr);
        }
        double t1 = Timer.getFPGATimestamp();
        System.out.printf("5: duration sec %5.3f\n", (t1 - t0));
        System.out.printf("5: duration per row us %5.3f\n", 1000000 * (t1 - t0) / N);
        channel.close();
    }

    /**
     * Channel.send (non-blocking) with direct buffer, connect
     * 
     * 1.1 us per packet
     */
    @Test
    void test6() throws IOException {
        DatagramChannel channel = DatagramChannel.open();
        channel.configureBlocking(false);
        channel.setOption(StandardSocketOptions.SO_SNDBUF, SOCKET_BUF_SIZE);
        InetSocketAddress sockAddr = new InetSocketAddress(m_addr, PORT);
        channel.connect(sockAddr);
        ByteBuffer m_bb = ByteBuffer.allocateDirect(PACKET_BUF_SIZE);
        double t0 = Timer.getFPGATimestamp();
        for (int i = 0; i < N; ++i) {
            m_bb.rewind();
            channel.write(m_bb);
        }
        double t1 = Timer.getFPGATimestamp();
        System.out.printf("6: duration sec %5.3f\n", (t1 - t0));
        System.out.printf("6: duration per row us %5.3f\n", 1000000 * (t1 - t0) / N);
        channel.close();
    }
}
