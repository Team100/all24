package org.team100.lib.telemetry;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.HashMap;
import java.util.Map;

import org.junit.jupiter.api.Test;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StringSubscriber;

class TelemetryTest {
    @Test
    void testKeySyntax() {
        NetworkTableInstance inst = NetworkTableInstance.create();
        Map<String, Publisher> publishers = new HashMap<>();
        Telemetry t = new Telemetry(inst, publishers);
        assertThrows(IllegalArgumentException.class, () -> t.log("", 1.0));
        assertThrows(IllegalArgumentException.class, () -> t.log("foo", 1.0));
    }

    @Test
    void testValueTypeClash() {
        NetworkTableInstance inst = NetworkTableInstance.create();
        Map<String, Publisher> publishers = new HashMap<>();
        Telemetry t = new Telemetry(inst, publishers);
        t.log("/foo", 1.0);
        assertThrows(IllegalArgumentException.class, () -> t.log("/foo", true));
    }

    @Test
    void testBoolean() {
        NetworkTableInstance inst = NetworkTableInstance.create();
        BooleanSubscriber sub = inst.getBooleanTopic("/foo").subscribe(false);
        assertEquals(false, sub.get());
        Map<String, Publisher> publishers = new HashMap<>();
        Telemetry t = new Telemetry(inst, publishers);
        t.log("/foo", true);
        assertEquals(true, sub.get());
    }

    @Test
    void testDouble() {
        NetworkTableInstance inst = NetworkTableInstance.create();
        DoubleSubscriber sub = inst.getDoubleTopic("/foo").subscribe(0);
        assertEquals(0.0, sub.get());
        Map<String, Publisher> publishers = new HashMap<>();
        Telemetry t = new Telemetry(inst, publishers);
        t.log("/foo", 1.0);
        assertEquals(1.0, sub.get());
    }

    @Test
    void testInteger() {
        NetworkTableInstance inst = NetworkTableInstance.create();
        IntegerSubscriber sub = inst.getIntegerTopic("/foo").subscribe(0);
        assertEquals(0.0, sub.get());
        Map<String, Publisher> publishers = new HashMap<>();
        Telemetry t = new Telemetry(inst, publishers);
        t.log("/foo", 1);
        assertEquals(1, sub.get());
    }

    @Test
    void testString() {
        NetworkTableInstance inst = NetworkTableInstance.create();
        StringSubscriber sub = inst.getStringTopic("/foo").subscribe("");
        assertEquals("", sub.get());
        Map<String, Publisher> publishers = new HashMap<>();
        Telemetry t = new Telemetry(inst, publishers);
        t.log("/foo", "hello");
        assertEquals("hello", sub.get());
    }

    @Test
    void testStringArray() {
        NetworkTableInstance inst = NetworkTableInstance.create();
        StringArraySubscriber sub = inst.getStringArrayTopic("/foo").subscribe(new String[0]);
        assertArrayEquals(new String[0], sub.get());
        Map<String, Publisher> publishers = new HashMap<>();
        Telemetry t = new Telemetry(inst, publishers);
        t.log("/foo", new String[] { "one", "two" });
        assertArrayEquals(new String[] { "one", "two" }, sub.get());
    }
}
