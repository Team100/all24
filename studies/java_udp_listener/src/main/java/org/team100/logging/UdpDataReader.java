package org.team100.logging;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;

import org.team100.lib.telemetry.UdpPrimitiveProtocol2;
import org.team100.lib.telemetry.UdpPrimitiveProtocol2.ProtocolException;
import org.team100.lib.telemetry.UdpSender;

public class UdpDataReader implements Runnable {
    private final UdpDataDecoder m_decoder;

    /** nullable */
    private final DatagramChannel m_channel;

    /** TODO: maybe this should be the "protocol"'s buffer' */
    private final ByteBuffer m_buffer;

    public UdpDataReader(UdpDataDecoder decoder) {
        m_decoder = decoder;
        m_channel = makeChannel(UdpSender.kPort);
        m_buffer = ByteBuffer.allocateDirect(UdpSender.MTU);
        // big-endian is the default, but just to make it clear...
        m_buffer.order(ByteOrder.BIG_ENDIAN);
    }

    @Override
    public void run() {
        System.out.println("data reader running...");
        while (true) {
            try {
                m_buffer.clear();
                m_channel.receive(m_buffer);
                m_buffer.limit(m_buffer.position());
                m_buffer.position(0);
                // System.out.printf("read data buffer position %d remaining %d\n", m_buffer.position(), m_buffer.remaining());
                // packet starts with timestamp
                long timestamp = UdpPrimitiveProtocol2.decodeLong(m_buffer);
                while (m_buffer.remaining() > 0) {
                    m_decoder.decode(m_buffer);
                }
            } catch (IOException | ProtocolException e) {
                e.printStackTrace();
            }
        }
    }

    private static DatagramChannel makeChannel(int port) {
        try {
            DatagramChannel channel = DatagramChannel.open();
            channel.configureBlocking(true);
            InetSocketAddress sockAddr = new InetSocketAddress(port);
            channel.bind(sockAddr);
            return channel;
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

}
