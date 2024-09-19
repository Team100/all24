package org.team100.logging;

public class UdpLogBridge {
    public static void main(String[] args) {
        try {
            //UdpConsumersInterface consumers = new UdpConsumers();
            UdpConsumersInterface consumers = new DummyUdpConsumers();
            UdpDataDecoder dataDecoder = new UdpDataDecoder(consumers);
            UdpMetaDecoder metaDecoder = new UdpMetaDecoder(consumers);
            UdpDataReader dataReader = new UdpDataReader(dataDecoder);
            UdpMetaReader metaReader = new UdpMetaReader(metaDecoder);
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
