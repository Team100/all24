package org.team100.logging;

import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.WPIUtilJNI;

public class UdpLogBridge {
    /**
     * For the little "Run" button below to work, you need to add this line to
     * VSCode's launch.json file:
     * 
     * "vmArgs": "-Djava.library.path=build/jni/release"
     */
    public static void main(String[] args) {
        // cribbed from StandaloneAppSamples
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(true);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(true);
        WPIMathJNI.Helper.setExtractOnStaticLoad(true);
        CameraServerJNI.Helper.setExtractOnStaticLoad(true);
        CameraServerCvJNI.Helper.setExtractOnStaticLoad(true);
        try {
            // sender can go about 30M keys/sec.
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
            metaReaderThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // TODO: never exit
    }
}
