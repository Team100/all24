package org.team100.lib.logging;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.function.Consumer;

public class UdpSender implements Consumer<ByteBuffer> {
    /**
     * 508 is the "really don't fragment" size. Our network uses a 1500 byte MTU so
     * 1472 is probably just as good. We might be able to set up both ends with
     * jumbo frames (8kb) which might be even better, and it could also be that the
     * packet loss rate is so low that fragmenting and reassembling is no problem
     * either.
     */
    public static final int MTU = 1472;
    private static final byte[] ADDR = new byte[] { 10, 1, 0, 16 };
    // TODO: rearrange packages to make these package-private
    public static final int kPort = 1995;
    public static final int kmetadataPort = 1996;
    /** nullable */
    private final DatagramChannel m_channel;

    private int m_counter;

    public UdpSender(int port) {
        m_channel = makeChannel(port);
        m_counter = 0;
    }

    public int getCounter() {
        return m_counter;
    }

    public static UdpSender data() {
        return new UdpSender(kPort);
    }

    public static UdpSender meta() {
        return new UdpSender(kmetadataPort);
    }

    private static DatagramChannel makeChannel(int port) {
        try {
            DatagramChannel channel = DatagramChannel.open();
            channel.configureBlocking(false);
            // big buffer does not help but doesn't hurt
            channel.setOption(StandardSocketOptions.SO_SNDBUF, 1000000);

            InetAddress m_addr = InetAddress.getByAddress(ADDR);

            // NOTE! don't use localhost in prod! for testing!
            // InetAddress m_addr = InetAddress.getLocalHost();

            InetSocketAddress sockAddr = new InetSocketAddress(m_addr, port);
            // this will fail for localhost if there's no listener
            channel.connect(sockAddr);
            return channel;
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    @Override
    public void accept(ByteBuffer bb) {
        if (m_channel == null) {
            System.out.println("no channel");
            return;
        }
        try {
            // should write bb.remaining() bytes.
            // int bytesWritten = m_channel.write(bb);
            m_channel.write(bb);
            // m_counter++;
            // System.out.println("counter " + m_counter);
            // System.out.println("bytes " + bytesWritten);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

}
