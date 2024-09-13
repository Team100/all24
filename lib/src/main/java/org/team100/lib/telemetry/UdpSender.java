package org.team100.lib.telemetry;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.function.Consumer;

public class UdpSender implements Consumer<ByteBuffer> {
    private static final byte[] ADDR = new byte[] { 10, 1, 0, 16 };
    private static final int kPort = 1995;
    /** nullable */
    private final DatagramChannel m_channel;

    public UdpSender() {
        m_channel = makeChannel();
    }

    private static DatagramChannel makeChannel() {
        try {
            DatagramChannel channel = DatagramChannel.open();
            channel.configureBlocking(false);
            // big buffer does not help but doesn't hurt
            channel.setOption(StandardSocketOptions.SO_SNDBUF, 1000000);

            InetAddress m_addr = InetAddress.getByAddress(ADDR);
            // InetAddress m_addr = InetAddress.getLocalHost();

            InetSocketAddress sockAddr = new InetSocketAddress(m_addr, kPort);
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
        if (m_channel == null)
            return;
        try {
            m_channel.write(bb);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

}
