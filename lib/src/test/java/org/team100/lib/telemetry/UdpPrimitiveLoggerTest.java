package org.team100.lib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.net.StandardSocketOptions;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.Timer;

class UdpPrimitiveLoggerTest {
    /** Send some examples. */
    @Test
    void testSending() throws UnknownHostException, SocketException {
        UdpPrimitiveLogger udpLogger = new UdpPrimitiveLogger();
        SupplierLogger logger = new SupplierLogger(
                Telemetry.get(),
                "root",
                () -> true,
                udpLogger,
                () -> false,
                null);
        for (int i = 0; i < 100; ++i) {
            logger.logBoolean(Level.COMP, "boolkey", () -> true);
            logger.logDouble(Level.COMP, "doublekey", () -> 100.0);
            logger.logInt(Level.COMP, "intkey", () -> 100);
            logger.logDoubleArray(Level.COMP, "doublearraykey", () -> new double[] { 1.0, 2.0 });
            logger.logDoubleObjArray(Level.COMP, "doubleobjarraykey", () -> new Double[] { 1.0, 2.0 });
            logger.logLong(Level.COMP, "longkey", () -> (long) 100);
            logger.logString(Level.COMP, "stringkey", () -> "value");
        }
        udpLogger.flush();
        assertEquals(8, udpLogger.handles.size());
        for (String k : udpLogger.handles.keySet()) {
            System.out.println(k);
        }
        assertEquals(1, udpLogger.handles.get("/root/boolkey"));
        assertEquals(2, udpLogger.handles.get("/root/doublekey"));
        assertEquals(3, udpLogger.handles.get("/root/intkey"));
        assertEquals(4, udpLogger.handles.get("/root/doublearraykey"));
        assertEquals(5, udpLogger.handles.get("/root/doubleobjarraykey"));
        assertEquals(6, udpLogger.handles.get("/root/longkey"));
        assertEquals(7, udpLogger.handles.get("/root/stringkey"));
    }

    /**
     * Send at a realistic rate (50 hz), with as many keys as possible.
     * 
     * How many keys is possible?
     * 
     * About 5000, this produces 15ms per cycle of socket.send() waiting.
     * 
     * This produces a peak packet rate of almost 300k PPS, which is about the
     * maximum possible.
     * 
     * With a python receiver also on my desktop, the loss rate at 300k PPS is
     * quite low, about 1%.
     * 
     * The encoding part of flushing is not the bottleneck: these results are the
     * same if the encoding is removed.
     * 
     * DatagramSocket and DatagramChannel behave about the same.
     * 
     * socket.connect() doesn't help.
     * 
     * @throws InterruptedException
     */
    @Test
    void testAWholeLot() throws UnknownHostException, SocketException, InterruptedException {
        UdpPrimitiveLogger udpLogger = new UdpPrimitiveLogger();
        SupplierLogger logger = new SupplierLogger(
                Telemetry.get(),
                "root",
                () -> true,
                udpLogger,
                () -> false,
                null);

        double t0 = Timer.getFPGATimestamp();
        final double interval = 0.02;
        final double total_time = 2;
        final int keys = 5000;
        final double expected_keys_per_sec = keys / interval;
        System.out.println("expected keys per second: " + expected_keys_per_sec);
        for (int i = 0; i < (total_time / interval); ++i) {
            double d = Timer.getFPGATimestamp() - t0;
            double dt = interval - (d % interval);
            Thread.sleep((long) (dt * 1000) + 1);
            d = Timer.getFPGATimestamp() - t0;
            double t1 = Timer.getFPGATimestamp();
            for (int j = 0; j < keys; ++j) {
                double val = Math.sin(j + 0.01 * i);
                logger.logDouble(Level.COMP, "doublekey" + j, () -> val);
            }
            udpLogger.flush();
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("et %.3f\n", t2 - t1);
        }

    }

    /**
     * Socket.send
     * 
     * 2.1 us per row (no listener)
     */
    @Test
    void testSocket() throws IOException {
        InetAddress m_addr = InetAddress.getByAddress(new byte[] { 10, 1, 0, 16 });
        DatagramSocket m_socket = new DatagramSocket();
        m_socket.setSendBufferSize(100000000);
        byte[] m_bytes = new byte[30];
        DatagramPacket p = new DatagramPacket(m_bytes, 30, m_addr, 1995);
        double t0 = Timer.getFPGATimestamp();
        int N = 1000000;
        for (int i = 0; i < N; ++i) {
            m_socket.send(p);
        }
        m_socket.close();
        double t1 = Timer.getFPGATimestamp();
        System.out.printf("duration sec %5.3f\n", (t1 - t0));
        System.out.printf("duration per row us %5.3f\n", 1000000 * (t1 - t0) / N);
    }

    /**
     * Socket.connect and then socket.send.
     * 
     * 2.1 us per row (no listener)
     */
    @Test
    void testSocketWithConnect() throws IOException {
        InetAddress m_addr = InetAddress.getByAddress(new byte[] { 10, 1, 0, 16 });
        DatagramSocket m_socket = new DatagramSocket();
        m_socket.setSendBufferSize(100000000);
        m_socket.connect(m_addr, 1995);
        byte[] m_bytes = new byte[30];
        DatagramPacket p = new DatagramPacket(m_bytes, 30, m_addr, 1995);
        double t0 = Timer.getFPGATimestamp();
        int N = 1000000;
        for (int i = 0; i < N; ++i) {
            m_socket.send(p);
        }
        m_socket.close();
        double t1 = Timer.getFPGATimestamp();
        System.out.printf("duration sec %5.3f\n", (t1 - t0));
        System.out.printf("duration per row us %5.3f\n", 1000000 * (t1 - t0) / N);
    }

    /**
     * Channel.connect and then channel.write (non-blocking)
     * 
     * 2.0 us per row (no listener)
     */
    @Test
    void testChannel() throws IOException {
        DatagramChannel channel = DatagramChannel.open();
        channel.configureBlocking(false);
        channel.setOption(StandardSocketOptions.SO_SNDBUF, 1000000000);
        InetAddress m_addr = InetAddress.getByAddress(new byte[] { 10, 1, 0, 16 });
        InetSocketAddress sockAddr = new InetSocketAddress(m_addr, 1995);
        channel.connect(sockAddr);
        byte[] m_bytes = new byte[30];
        ByteBuffer m_bb = ByteBuffer.wrap(m_bytes);
        double t0 = Timer.getFPGATimestamp();
        int N = 1000000;
        for (int i = 0; i < N; ++i) {
            m_bb.rewind();
            channel.write(m_bb);
        }
        channel.close();
        double t1 = Timer.getFPGATimestamp();
        System.out.printf("duration sec %5.3f\n", (t1 - t0));
        System.out.printf("duration per row us %5.3f\n", 1000000 * (t1 - t0) / N);
    }

    /**
     * Channel.send (non-blocking)
     * 
     * 2.2 us per row (no listener)
     */
    @Test
    void testChannelWithSendAndNotConnect() throws IOException {
        DatagramChannel channel = DatagramChannel.open();
        channel.configureBlocking(false);
        channel.setOption(StandardSocketOptions.SO_SNDBUF, 1000000000);
        InetAddress m_addr = InetAddress.getByAddress(new byte[] { 10, 1, 0, 16 });
        InetSocketAddress sockAddr = new InetSocketAddress(m_addr, 1995);
        byte[] m_bytes = new byte[30];
        ByteBuffer m_bb = ByteBuffer.wrap(m_bytes);
        double t0 = Timer.getFPGATimestamp();
        int N = 1000000;
        for (int i = 0; i < N; ++i) {
            m_bb.rewind();
            channel.send(m_bb, sockAddr);
        }
        channel.close();
        double t1 = Timer.getFPGATimestamp();
        System.out.printf("duration sec %5.3f\n", (t1 - t0));
        System.out.printf("duration per row us %5.3f\n", 1000000 * (t1 - t0) / N);
    }
}
