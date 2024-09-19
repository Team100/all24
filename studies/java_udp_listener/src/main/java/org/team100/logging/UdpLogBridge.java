package org.team100.logging;

public class UdpLogBridge {
    public static void main(String[] args) {
        try {
            UdpDataReader dataReader = new UdpDataReader();
            UdpMetaReader metaReader = new UdpMetaReader();
            Thread dataReaderThread = new Thread(dataReader);
            dataReaderThread.start();
            Thread metaReaderThread = new Thread(metaReader);
            metaReaderThread.start();
            // these should block forever.
            dataReaderThread.join();
            metaReaderThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // TODO: never exit
    }
}
