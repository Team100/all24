package org.team100.logging;

import java.io.IOException;
import java.nio.ByteBuffer;

import org.opencv.core.Core;

import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;

public final class Main {
    private Main() {
    }

    public static void main(String... args) throws IOException, InterruptedException {
        // cribbed from StandaloneAppSamples
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(true);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(true);
        WPIMathJNI.Helper.setExtractOnStaticLoad(true);
        CameraServerJNI.Helper.setExtractOnStaticLoad(true);
        CameraServerCvJNI.Helper.setExtractOnStaticLoad(true);

        // this doesn't work on the pi.
        // CombinedRuntimeLoader.loadLibraries(Main.class, "wpiutiljni", "wpimathjni", "ntcorejni",
        //         Core.NATIVE_LIBRARY_NAME, "cscorejni");

        System.out.println("HELLO");
        testDoubleEncodingPerformance();
        var inst = NetworkTableInstance.getDefault();
        inst.startServer();
        var foo = inst.getDoubleTopic("foo");
        for (int i = 0; i < 100; ++i) {
            System.out.println(i);
            foo.publish().set(1.0);
            Thread.sleep(1000);
        }
    }

    static void testDoubleEncodingPerformance() {
        // this goes at 1.3 ns/row, with or without offset
        final int ITERATIONS = 10000000;
        final int N = 180;
        ByteBuffer bb = ByteBuffer.allocateDirect(N * 8);
        // Timer needs HAL, which does not exist.
        // double t1 = Timer.getFPGATimestamp();
        double t1 = System.nanoTime()/1e9;
        for (int i = 0; i < ITERATIONS; ++i) {
            bb.clear();
            for (int j = 0; j < N; ++j) {
                bb.putDouble(j, (double) j * 8);
            }
        }
        // double t2 = Timer.getFPGATimestamp();
        double t2 = System.nanoTime()/1e9;
        System.out.printf("string duration sec %.3f\n", t2 - t1);
        System.out.printf("string duration per row ns %.3f\n", 1e9 * (t2 - t1) / (ITERATIONS * N));
    }
}
