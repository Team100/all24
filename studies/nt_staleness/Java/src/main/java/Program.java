
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.CombinedRuntimeLoader;

import java.io.IOException;
import java.util.EnumSet;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * Program
 */
public class Program {
    public static void main(String[] args) throws IOException, InterruptedException {
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);
        CameraServerJNI.Helper.setExtractOnStaticLoad(false);
        CameraServerCvJNI.Helper.setExtractOnStaticLoad(false);

        CombinedRuntimeLoader.loadLibraries(Program.class, "wpiutiljni", "wpimathjni", "ntcorejni",
                Core.NATIVE_LIBRARY_NAME, "cscorejni");

        var inst = NetworkTableInstance.getDefault();
        inst.startServer();
        inst.addListener(new String[] { "" }, EnumSet.of(NetworkTableEvent.Kind.kValueAll), Program::apply);

        while (true) {
            Thread.sleep(1000);
            System.out.println("ping");
        }
    }

    private static void apply(NetworkTableEvent e) {
        ValueEventData ve = e.valueData;
        // NetworkTableValue v = ve.value;
        String name = ve.getTopic().getName();
        System.out.println(name);

    }
}
