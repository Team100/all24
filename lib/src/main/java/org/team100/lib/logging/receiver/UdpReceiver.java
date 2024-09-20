package org.team100.lib.logging.receiver;

public class UdpReceiver {

    public static void run() throws InterruptedException {
        // sender can go about 30M keys/sec.
        // real consumer can go about 4M keys/sec.
        UdpConsumersInterface consumers = new UdpConsumers();
        // the dummy consumer can keep up, about 30M keys/sec
        // UdpConsumersInterface consumers = new DummyUdpConsumers();
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
        System.out.println("done");
        consumers.close();
    }

}
