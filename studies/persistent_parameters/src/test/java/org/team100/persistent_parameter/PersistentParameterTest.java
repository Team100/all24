package org.team100.persistent_parameter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.nio.file.Path;

import org.junit.jupiter.api.Test;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;

class PersistentParameterTest {
    private static final double kDelta = 0.001;

    private static void init(NetworkTableInstance inst) {
        String cwd = Path.of("").toAbsolutePath().toString();
        if (cwd.endsWith("release")) {
            // vscode test runner cwd is build/jni/release
            inst.startServer("../../../nttest.json");
        } else if (cwd.endsWith("persistent_parameters")) {
            // wpilib test runner cwd is project root
            inst.startServer("nttest.json");
        } else {
            fail("weird cwd: " + cwd);
        }
        Preferences.setNetworkTableInstance(inst);
        try {
            int count = 0;
            // The data file has the key "foo" in it, so wait until that key appears,
            // indicating the server is done reading the file.
            while (!Preferences.containsKey("foo")) {
                Thread.sleep(100);
                count++;
                if (count > 30) {
                    throw new InterruptedException();
                }
            }
        } catch (InterruptedException ex) {
            fail("interrupted while waiting for server to start");
        }
    }

    @Test
    void testPersistentParameter() throws InterruptedException {
        NetworkTableInstance inst = NetworkTableInstance.create();
        init(inst);

        // supply a default of 1.0 here:
        PersistentParameter p = new PersistentParameter("foo", 1.0, () -> 0.0);

        // but the data file has 2.0, which takes precedence.
        assertEquals(2.0, p.get(), kDelta);

        // write a different value and verify it
        p.set(3.0);
        assertEquals(3.0, p.get(), kDelta);

        // set it back to what it was.
        p.set(2.0);

        // and write the file again in case the value above was written.
        inst.flush();

        // flushing is asynchronous, so wait a bit.
        Thread.sleep(1000);
    }
}
