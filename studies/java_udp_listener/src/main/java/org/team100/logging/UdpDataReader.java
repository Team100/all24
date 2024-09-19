package org.team100.logging;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;

import org.team100.lib.telemetry.UdpPrimitiveProtocol2;
import org.team100.lib.telemetry.UdpSender;
import org.team100.lib.telemetry.UdpType;

public class UdpDataReader implements Runnable {
    // TODO: consolidate these flags somewhere
    // write to network tables
    private static final boolean PUB = false;
    // write to disk
    private static final boolean LOG = false;

    /** nullable */
    private final DatagramChannel m_channel;

    /** TODO: maybe this should be the "protocol"'s buffer' */
    private final ByteBuffer m_buffer;

    public UdpDataReader() {
        m_channel = makeChannel(UdpSender.kPort);
        m_buffer = ByteBuffer.allocateDirect(UdpSender.MTU);
        // big-endian is the default, but just to make it clear...
        m_buffer.order(ByteOrder.BIG_ENDIAN);
    }

    long timestamp;
    int key;
    UdpType type;

    @Override
    public void run() {
        while (true) {
            try {
                int bytesRead = m_channel.read(m_buffer);
                // TODO: something with bytesread
                int offset = 0;
                while (offset < bytesRead) {
                    offset = UdpPrimitiveProtocol2.decodeLong(m_buffer, offset, x -> timestamp = x);
                    offset = UdpPrimitiveProtocol2.decodeKey(m_buffer, offset, x -> key = x);
                    offset = UdpPrimitiveProtocol2.decodeType(m_buffer, offset, x -> type = x);
                    switch (type) {
                        case BOOLEAN -> offset = UdpPrimitiveProtocol2.decodeBoolean();
                        case DOUBLE -> offset = UdpPrimitiveProtocol2.decodeBoolean();
                        case INT -> offset = UdpPrimitiveProtocol2.decodeBoolean();
                        case DOUBLE_ARRAY -> offset = UdpPrimitiveProtocol2.decodeBoolean();
                        case LONG -> offset = UdpPrimitiveProtocol2.decodeBoolean();
                        case STRING -> offset = UdpPrimitiveProtocol2.decodeBoolean();
                        default -> System.out.println("blarg");
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private static DatagramChannel makeChannel(int port) {
        try {
            DatagramChannel channel = DatagramChannel.open();
            InetSocketAddress sockAddr = new InetSocketAddress(port);
            channel.connect(sockAddr);
            return channel;
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

}
