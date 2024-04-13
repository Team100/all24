package server;

import java.io.IOException;
import java.util.EnumSet;

import org.opencv.core.Core;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.struct.StructBuffer;

/**
 * Program
 */
public class Program {
    private static final StructBuffer<Rotation3d> m_buf = StructBuffer.create(Rotation3d.struct);

    public static void main(String[] args) throws IOException, InterruptedException {
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);

        CombinedRuntimeLoader.loadLibraries(
                Program.class,
                "wpiutiljni",
                "ntcorejni",
                Core.NATIVE_LIBRARY_NAME);

        var inst = NetworkTableInstance.getDefault();
        inst.startServer();

        StructArrayPublisher<Rotation3d> pub = inst.getStructArrayTopic("pong", Rotation3d.struct)
                .publish(PubSubOption.keepDuplicates(true));

        MultiSubscriber sub = new MultiSubscriber(inst, new String[] { "ping" }, PubSubOption.keepDuplicates(true));
        inst.addListener(sub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), Program::apply);

        while (true) {
            Thread.sleep(1000);
            System.out.println("sending pong");
            Rotation3d[] poses = new Rotation3d[] {
                    new Rotation3d(6, 5, 4),
                    new Rotation3d(3, 2, 1) };
            pub.set(poses);
        }
    }

    private static void apply(NetworkTableEvent e) {
        ValueEventData ve = e.valueData;
        NetworkTableValue v = ve.value;
        String name = ve.getTopic().getName();
        byte[] b = v.getRaw();
        Rotation3d[] rots;
        try {
            synchronized (m_buf) {
                rots = m_buf.readArray(b);
            }
        } catch (RuntimeException ex) {
            System.out.println(ex.getMessage());
            return;
        }
        System.out.printf("receiving: %s length: %d\n", name, rots.length);
        for (Rotation3d r : rots) {
            System.out.printf("        %s\n", r);
        }
    }
}
