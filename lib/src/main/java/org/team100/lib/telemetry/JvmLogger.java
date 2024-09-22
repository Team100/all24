package org.team100.lib.telemetry;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryPoolMXBean;
import java.lang.management.MemoryUsage;
import java.util.HashMap;
import java.util.Map;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.LongSupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;

/**
 * Logs stuff about the JVM. Inspired by Advantage Kit's
 * LoggedRobot.GcStatsCollector().
 */
public class JvmLogger implements Glassy {
    private final Map<String, Long> times;
    private final Map<String, Long> counts;
    // LOGGERS
    private final LongSupplierLogger2 m_log_heap;
    private final LongSupplierLogger2 m_log_nonheap;
    private final LongSupplierLogger2 m_log_memory_total;
    private final LongSupplierLogger2 m_log_gc_time;
    private final LongSupplierLogger2 m_log_gc_count;

    public JvmLogger(SupplierLogger2 parent) {
        SupplierLogger2 child = parent.child(this);
        times = new HashMap<>();
        counts = new HashMap<>();
        m_log_heap = child.longLogger(Level.COMP, "MemoryUsage/heap");
        m_log_nonheap = child.longLogger(Level.COMP, "MemoryUsage/non-heap");
        m_log_memory_total = child.longLogger(Level.COMP, "MemoryPool/total");
        m_log_gc_time = child.longLogger(Level.TRACE, "GCTimeMS/total");
        m_log_gc_count = child.longLogger(Level.TRACE, "GCCounts/total");
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
            accumTime += thisTime;
            accumCount += thisCount;
        }
        long finalAccumTime = accumTime;
        long finalAccumCount = accumCount;
        m_log_gc_time.log(() -> finalAccumTime);
        m_log_gc_count.log(() -> finalAccumCount);
    }

    public void logMemoryPools() {
        long accumUsage = 0;
        for (MemoryPoolMXBean bean : ManagementFactory.getMemoryPoolMXBeans()) {
            MemoryUsage usage = bean.getUsage();
            accumUsage += usage.getUsed();
        }
        long finalAccumUsage = accumUsage;
        m_log_memory_total.log(() -> finalAccumUsage);
    }

    public void logMemoryUsage() {
        MemoryMXBean bean = ManagementFactory.getMemoryMXBean();
        m_log_heap.log(() -> bean.getHeapMemoryUsage().getUsed());
        m_log_nonheap.log(() -> bean.getNonHeapMemoryUsage().getUsed());
    }

    @Override
    public String getGlassName() {
        return "JVM Logger";
    }

}
