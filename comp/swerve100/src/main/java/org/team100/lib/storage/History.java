package org.team100.lib.storage;

import java.util.Collections;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

/** Keeps N past states, for when bitemporality is not required. */
public class History<Value> {
    private static final boolean debug = false;
    private final int capacity;
    private final NavigableMap<Double, Value> valid;
    private int size;

    public History(int capacity) {
        this.capacity = capacity;
        size = 0;
        valid = new ConcurrentSkipListMap<>();
    }

    /**
     * @param validTime represents the real-world time the value describes, in
     *                  seconds. the most-recent entries are retained, up to the
     *                  capacity.
     * @param value     value to store
     */
    public synchronized void put(double validTime, Value value) {
        // fix the key to avoid overwriting anything.
        while (valid.containsKey(validTime)) {
            validTime = Math.nextUp(validTime); // add smallest possible double
        }
        if (debug)
            System.out.println("put " + validTime + " " + value);
        valid.put(validTime, value);
        if (++size > capacity) {
            valid.pollFirstEntry();
            --size;
        }
    }

    public Value get(double validTime) {
        return valid.get(validTime);
    }

    /**
     * Find the most-recent value earlier than, or equal to, the specified valid
     * time, or null if no such entry exists.
     */
    public Entry<Double, Value> floor(double validTimeSec) {
        if (validTimeSec < 0)
            throw new IllegalArgumentException("Negative time is not allowed: " + validTimeSec);
        return validFloorEntry(validTimeSec);
    }

    public Value floorValue(double validTimeSec) {
        return floor(validTimeSec).getValue();
    }

    public NavigableMap<Double, Value> validSubMap(double fromTime, double toTime) {
        return Collections.unmodifiableNavigableMap(valid.subMap(fromTime, true, toTime, true));
    }

    public NavigableMap<Double, Value> validTailMap(double vt) {
        return Collections.unmodifiableNavigableMap(mutableValidTailMap(vt));
    }

    /** Find the entry for the greatest key less than or equal to vt. */
    public Entry<Double, Value> validFloorEntry(double vt) {
        return valid.floorEntry(vt);
    }

    public int size() {
        return size;
    }

    NavigableMap<Double, Value> mutableValidTailMap(double vt) {
        return valid.tailMap(vt, true);
    }

}
