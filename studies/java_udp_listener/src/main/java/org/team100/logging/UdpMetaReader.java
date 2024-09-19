package org.team100.logging;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;

import org.team100.lib.telemetry.UdpSender;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;

public class UdpMetaReader implements Runnable {
    // write to network tables
    private static final boolean PUB = false;
    // write to disk
    private static final boolean LOG = false;

    /** nullable */
    private final DatagramChannel m_channel;

    /** TODO: maybe this should be the "protocol"'s buffer' */
    private final ByteBuffer m_buffer;

    public UdpMetaReader() {
        m_channel = makeChannel(UdpSender.kmetadataPort);
        m_buffer = ByteBuffer.allocateDirect(UdpSender.MTU);
        // big-endian is the default, but just to make it clear...
        m_buffer.order(ByteOrder.BIG_ENDIAN);

        if (PUB) {
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            inst.startServer();
        }
        if (LOG) {
            // log_file = DataLog(dir=LOG_DIR, filename=LOG_FILENAME)
            DataLog log_file = new DataLog("", "", 0.1);
        }
    }

    @Override
    public void run() {
        while (true) {
            try {
                int bytesRead = m_channel.read(m_buffer);


                
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
