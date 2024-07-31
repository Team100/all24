package org.team100.lib.telemetry;

import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

/**
 * Send logs to a log recipient via UDP.
 * 
 * UDP is not formally reliable but on the robot LAN, for log data, it's good
 * enough.
 * 
 * The recipient IP is always 10.1.0.100.
 */
public class UdpPrimitiveLogger extends PrimitiveLogger {

    private final InetAddress m_addr;
    private final DatagramSocket m_socket;

    public UdpPrimitiveLogger() throws UnknownHostException, SocketException {
        m_addr = InetAddress.getByAddress(new byte[] { 10, 1, 0, 100 });
        m_socket = new DatagramSocket();
    }

    @Override
    void logBoolean(String key, boolean val) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logBoolean'");
    }

    @Override
    void logDouble(String key, double val) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logDouble'");
    }

    @Override
    void logInt(String key, int val) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logInt'");
    }

    @Override
    void logFloat(String key, float val) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logFloat'");
    }

    @Override
    void logDoubleArray(String key, double[] val) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logDoubleArray'");
    }

    @Override
    void logDoubleObjArray(String key, Double[] val) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logDoubleObjArray'");
    }

    @Override
    void logLong(String key, long val) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logLong'");
    }

    @Override
    void logString(String key, String val) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logString'");
    }

}
