package org.team100.lib.telemetry;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryPoolMXBean;
import java.lang.management.MemoryUsage;
import java.util.HashMap;
import java.util.Map;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

/**
 * Logs stuff about the JVM. Inspired by Advantage Kit's
 * LoggedRobot.GcStatsCollector().
 */
public class JvmLogger implements Glassy {
    private final Telemetry.Logger t;
    private final String m_name;
    private final Map<String, Long> times;
    private final Map<String, Long> counts;

    public JvmLogger() {
        m_name = Names.name(this);
        t = Telemetry.get().logger(m_name);
        times = new HashMap<>();
        counts = new HashMap<>();
    }

    /** This doesn't seem to ever log anything. */
    public void logGarbageCollectors() {
        long accumTime = 0;
        long accumCount = 0;
        for (GarbageCollectorMXBean bean : ManagementFactory.getGarbageCollectorMXBeans()) {
            String pool = bean.getName();
            times.computeIfAbsent(pool, x -> 0l);
            counts.computeIfAbsent(pool, x -> 0l);
            long collectionTime = bean.getCollectionTime();
            long collectionCount = bean.getCollectionCount();
            long thisTime = collectionTime - times.get(pool);
            long thisCount = collectionCount - counts.get(pool);
            times.put(pool, collectionTime);
            counts.put(pool, collectionCount);
            t.log(Level.TRACE, "GCTimeMS/" + pool, thisTime);
            t.log(Level.TRACE, "GCCounts/" + pool, thisCount);
            accumTime += thisTime;
            accumCount += thisCount;
        }
        t.log(Level.TRACE, "GCTimeMS/total", accumTime);
        t.log(Level.TRACE, "GCCounts/total", accumCount);
    }

    public void logMemoryPools() {
        long accumUsage = 0;
        for (MemoryPoolMXBean bean : ManagementFactory.getMemoryPoolMXBeans()) {
            MemoryUsage usage = bean.getUsage();
            accumUsage += usage.getUsed();
            t.log(Level.INFO, "MemoryPool/" + bean.getName(), usage.getUsed());
        }
        t.log(Level.INFO, "MemoryPool/total", accumUsage);
    }

    public void logMemoryUsage() {
        MemoryMXBean bean = ManagementFactory.getMemoryMXBean();
        t.log(Level.INFO, "MemoryUsage/heap", bean.getHeapMemoryUsage().getUsed());
        t.log(Level.INFO, "MemoryUsage/non-heap", bean.getNonHeapMemoryUsage().getUsed());
    }

    @Override
    public String getGlassName() {
        return "JVM Logger";
    }

}
