package org.team100.logging;

import org.team100.lib.logging.receiver.UdpReceiver;

import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.WPIUtilJNI;

public final class Main {

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
            UdpReceiver.run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        System.out.println("done");
        // TODO: never exit
    }

    private Main() {
    }
}
