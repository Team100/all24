package org.team100.logging;

import java.util.Arrays;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.atomic.AtomicInteger;

import static java.util.concurrent.TimeUnit.SECONDS;

import org.team100.lib.telemetry.UdpType;

/** For testing */
public class DummyUdpConsumers implements UdpConsumersInterface {

    private static final boolean PRINT = false;
    private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
    AtomicInteger counter = new AtomicInteger(0);

    public DummyUdpConsumers() {
        if (PRINT)
            System.out.println("using dummy consumer");
        scheduler.scheduleAtFixedRate(
                () -> System.out.printf("counter %d\n", counter.getAndSet(0)),
                0, 1, SECONDS);
    }

    @Override
    public void acceptBoolean(int key, boolean val) {
        counter.incrementAndGet();
        if (PRINT)
            System.out.printf("bool key: %d value: %b\n", key, val);
    }

    @Override
    public void acceptDouble(int key, double val) {
        counter.incrementAndGet();
        if (PRINT)
            System.out.printf("double key: %d value: %f\n", key, val);
    }

    @Override
    public void acceptInt(int key, int val) {
        counter.incrementAndGet();
        if (PRINT)
            System.out.printf("int key: %d value: %d\n", key, val);
    }

    @Override
    public void acceptDoubleArray(int key, double[] val) {
        counter.incrementAndGet();
        if (PRINT)
            System.out.printf("double[] key: %d value: %s\n", key, Arrays.toString(val));
    }

    @Override
    public void acceptString(int key, String val) {
        counter.incrementAndGet();
        if (PRINT)
            System.out.printf("string key: %d value: %s\n", key, val);
    }

    @Override
    public void acceptMeta(int key, UdpType type, String val) {
        counter.incrementAndGet();
        if (PRINT)
            System.out.printf("META key: %d type: %s, value: %s\n", key, type.name(), val);
    }

}
