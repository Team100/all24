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

/**
 * Logs stuff about the JVM. Inspired by Advantage Kit's
 * LoggedRobot.GcStatsCollector().
 */
public class JvmLogger implements Glassy {
    private final SupplierLogger m_logger;
    private final Map<String, Long> times;
    private final Map<String, Long> counts;

    public JvmLogger(SupplierLogger parent) {
        m_logger = parent.child(this);
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
            m_logger.logLong(Level.TRACE, "GCTimeMS/" + pool, () -> thisTime);
            m_logger.logLong(Level.TRACE, "GCCounts/" + pool, () -> thisCount);
            accumTime += thisTime;
            accumCount += thisCount;
        }
        long finalAccumTime = accumTime;
        long finalAccumCount = accumCount;
        m_logger.logLong(Level.TRACE, "GCTimeMS/total", () -> finalAccumTime);
        m_logger.logLong(Level.TRACE, "GCCounts/total", () -> finalAccumCount);
    }

    public void logMemoryPools() {
        long accumUsage = 0;
        for (MemoryPoolMXBean bean : ManagementFactory.getMemoryPoolMXBeans()) {
            MemoryUsage usage = bean.getUsage();
            accumUsage += usage.getUsed();
            m_logger.logLong(Level.COMP, "MemoryPool/" + bean.getName(), usage::getUsed);
        }
        long finalAccumUsage = accumUsage;
        m_logger.logLong(Level.COMP, "MemoryPool/total", () -> finalAccumUsage);
    }

    public void logMemoryUsage() {
        MemoryMXBean bean = ManagementFactory.getMemoryMXBean();
        m_logger.logLong(Level.COMP, "MemoryUsage/heap", () -> bean.getHeapMemoryUsage().getUsed());
        m_logger.logLong(Level.COMP, "MemoryUsage/non-heap", () -> bean.getNonHeapMemoryUsage().getUsed());
    }

    @Override
    public String getGlassName() {
        return "JVM Logger";
    }

}
