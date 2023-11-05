package org.team100.lib.storage;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.SortedMap;

import org.junit.jupiter.api.Test;

class BitemporalBufferTest {

    @Test
    void testTailMapInclusion() {
        BitemporalBuffer<String> buf = new BitemporalBuffer<>(10);
        buf.put(0l, 0.0, "hello");
        SortedMap<Long, Entry<Double, String>> s1 = buf.recordTailMap(0l);
        SortedMap<Double, Entry<Long, String>> s2 = buf.validTailMap(0l);

        assertAll(
                () -> assertEquals(1, s1.size()),
                () -> assertEquals("hello", s1.get(0l).getValue()),
                () -> assertEquals(1, s2.size()),
                () -> assertEquals("hello", s2.get(0.0).getValue()));
    }

    @Test
    void testTailMapExclusion() {
        BitemporalBuffer<String> buf = new BitemporalBuffer<>(10);
        buf.put(0l, 0.0, "hello");
        SortedMap<Long, Entry<Double, String>> s3 = buf.recordTailMap(1l);
        SortedMap<Double, Entry<Long, String>> s4 = buf.validTailMap(1l);
        assertAll(
                () -> assertEquals(0, s3.size()),
                () -> assertEquals(0, s4.size()));
    }

    @Test
    void testDuplicateKeys() {
        BitemporalBuffer<String> buf = new BitemporalBuffer<>(10);
        // add duplicate keys
        buf.put(0l, 0.0, "hello");
        buf.put(0l, 0.0, "duplicate");

        NavigableMap<Long, Entry<Double, String>> s1 = buf.recordTailMap(0l);
        assertEquals(2, s1.size());
        Iterator<Entry<Long, Entry<Double, String>>> records = s1.entrySet().iterator();
        {
            // the first entry is here as entered
            Entry<Long, Entry<Double, String>> recordEntry = records.next();
            Entry<Double, String> recordValue = recordEntry.getValue();
            
            assertAll(
                    () -> assertEquals(0l, recordEntry.getKey()),
                    () -> assertEquals(0.0, recordValue.getKey()),
                    () -> assertEquals("hello", recordValue.getValue()));

        }
        {
            // the second entry has incremented keys
            Entry<Long, Entry<Double, String>> recordEntry = records.next();
            Entry<Double, String> recordValue = recordEntry.getValue();
            assertAll(
                    () -> assertEquals(1l, recordEntry.getKey()),
                    () -> assertEquals(4.9E-324, recordValue.getKey()),
                    () -> assertEquals("duplicate", recordValue.getValue()));
        }

        // the keys are the same in both maps
        NavigableMap<Double, Entry<Long, String>> s2 = buf.validTailMap(0l);
        assertEquals(2, s2.size());
        Iterator<Entry<Double, Entry<Long, String>>> valids = s2.entrySet().iterator();
        {
            Entry<Double, Entry<Long, String>> validEntry = valids.next();
            Entry<Long, String> validValue = validEntry.getValue();   
            assertAll(
                    () -> assertEquals(0.0, validEntry.getKey()),
                    () -> assertEquals(0l, validValue.getKey()),
                    () -> assertEquals("hello", validValue.getValue()));
        }
        {
            Entry<Double, Entry<Long, String>> validEntry = valids.next();
            Entry<Long, String> validValue = validEntry.getValue();
            assertAll(
                    () -> assertEquals(4.9E-324, validEntry.getKey()),
                    () -> assertEquals(1l, validValue.getKey()),
                    () -> assertEquals("duplicate", validValue.getValue()));
        }
    }

    @Test
    void testOrder() {
        BitemporalBuffer<String> buf = new BitemporalBuffer<>(4);
        // all out of order
        buf.put(2l, 1.0, "r2 v1");
        buf.put(1l, 2.0, "r1 v2");
        buf.put(0l, 3.0, "r0 v3");
        buf.put(3l, 0.0, "r3 v0");
        {
            NavigableMap<Long, Entry<Double, String>> recordTailMap = buf.recordTailMap(Long.MIN_VALUE);
            var recordValues = new ArrayList<>(recordTailMap.values());
            assertEquals(4, recordValues.size());
            // these should be in record order
            assertAll(
                    () -> assertEquals("r0 v3", recordValues.get(0).getValue()),
                    () -> assertEquals("r1 v2", recordValues.get(1).getValue()),
                    () -> assertEquals("r2 v1", recordValues.get(2).getValue()),
                    () -> assertEquals("r3 v0", recordValues.get(3).getValue()));
        }
        {
            NavigableMap<Double, Entry<Long, String>> validTailMap = buf.validTailMap(-Double.MAX_VALUE);
            var validValues = new ArrayList<>(validTailMap.values());
            assertEquals(4, validValues.size());
            assertAll(
                    // these should be in valid order
                    () -> assertEquals("r3 v0", validValues.get(0).getValue()),
                    () -> assertEquals("r2 v1", validValues.get(1).getValue()),
                    () -> assertEquals("r1 v2", validValues.get(2).getValue()),
                    () -> assertEquals("r0 v3", validValues.get(3).getValue()));
        }
    }

    @Test
    void testCapacity() {
        BitemporalBuffer<String> buf = new BitemporalBuffer<>(2);
        assertEquals(0, buf.size());
        {
            buf.put(0l, 0.0, "record 0");
            NavigableMap<Long, Entry<Double, String>> recordTailMap = buf.recordTailMap(0l);
            NavigableMap<Double, Entry<Long, String>> validTailMap = buf.validTailMap(0);
            assertAll(
                    () -> assertEquals(1, buf.size()),
                    () -> assertEquals(1, recordTailMap.size()),
                    () -> assertEquals("record 0", recordTailMap.firstEntry().getValue().getValue()),
                    () -> assertEquals(1, validTailMap.size()),
                    () -> assertEquals("record 0", validTailMap.firstEntry().getValue().getValue()));
        }
        {
            buf.put(1l, 1.0, "record 1");
            NavigableMap<Long, Entry<Double, String>> recordTailMap = buf.recordTailMap(1l);
            NavigableMap<Double, Entry<Long, String>> validTailMap = buf.validTailMap(1);
            assertAll(
                    () -> assertEquals(2, buf.size()),
                    () -> assertEquals(1, recordTailMap.size()),
                    () -> assertEquals("record 1", recordTailMap.firstEntry().getValue().getValue()),
                    () -> assertEquals(1, validTailMap.size()),
                    () -> assertEquals("record 1", validTailMap.firstEntry().getValue().getValue()));
        }
        {
            // this should bump record 0
            buf.put(2l, 2.0, "record 2");
            assertEquals(2, buf.size());
            {
                NavigableMap<Long, Entry<Double, String>> recordTailMap = buf.recordTailMap(2l);
                NavigableMap<Double, Entry<Long, String>> validTailMap = buf.validTailMap(2);
                assertAll(
                        () -> assertEquals(1, recordTailMap.size()),
                        () -> assertEquals("record 2", recordTailMap.firstEntry().getValue().getValue()),
                        () -> assertEquals(1, validTailMap.size()),
                        () -> assertEquals("record 2", validTailMap.firstEntry().getValue().getValue()));
            }
            {
                NavigableMap<Long, Entry<Double, String>> recordTailMap = buf.recordTailMap(Long.MIN_VALUE);
                NavigableMap<Double, Entry<Long, String>> validTailMap = buf.validTailMap(-Double.MAX_VALUE);
                assertAll(
                        () -> assertEquals(2, recordTailMap.size()),
                        () -> assertEquals("record 1", recordTailMap.firstEntry().getValue().getValue()),
                        () -> assertEquals(2, validTailMap.size()),
                        () -> assertEquals("record 1", validTailMap.firstEntry().getValue().getValue()));
            }
        }
    }

    @Test
    void testOutOfOrderCapacity() {
        BitemporalBuffer<String> buf = new BitemporalBuffer<>(2);
        assertEquals(0, buf.size());
        {
            buf.put(0l, 2.0, "r0 v2");
            assertEquals(1, buf.size());
            NavigableMap<Long, Entry<Double, String>> recordTailMap = buf.recordTailMap(0l);
            NavigableMap<Double, Entry<Long, String>> validTailMap = buf.validTailMap(2);
            assertAll(
                    () -> assertEquals(1, recordTailMap.size()),
                    () -> assertEquals("r0 v2", recordTailMap.firstEntry().getValue().getValue()),
                    () -> assertEquals(1, validTailMap.size()),
                    () -> assertEquals("r0 v2", validTailMap.firstEntry().getValue().getValue()));
        }
        {
            buf.put(1l, 0.0, "r1 v0");
            assertEquals(2, buf.size());
            NavigableMap<Long, Entry<Double, String>> recordTailMap = buf.recordTailMap(1l);
            NavigableMap<Double, Entry<Long, String>> validTailMap = buf.validTailMap(1);
            assertAll(
                    () -> assertEquals(1, recordTailMap.size()),
                    () -> assertEquals("r1 v0", recordTailMap.firstEntry().getValue().getValue()),
                    // this finds the earlier record representing the later valid time
                    () -> assertEquals(1, validTailMap.size()),
                    () -> assertEquals("r0 v2", validTailMap.firstEntry().getValue().getValue()));
        }
        {
            // this should bump record 1
            buf.put(2l, 1.0, "r2 v1");
            assertEquals(2, buf.size());
            {
                NavigableMap<Long, Entry<Double, String>> recordTailMap = buf.recordTailMap(2l);
                NavigableMap<Double, Entry<Long, String>> validTailMap = buf.validTailMap(1);
                assertAll(
                        () -> assertEquals(1, recordTailMap.size()),
                        () -> assertEquals("r2 v1", recordTailMap.firstEntry().getValue().getValue()),
                        () -> assertEquals(2, validTailMap.size()),
                        () -> assertEquals("r2 v1", validTailMap.firstEntry().getValue().getValue()),
                        () -> assertEquals("r0 v2", validTailMap.lastEntry().getValue().getValue()));
            }
            {
                NavigableMap<Long, Entry<Double, String>> recordTailMap = buf.recordTailMap(Long.MIN_VALUE);
                NavigableMap<Double, Entry<Long, String>> validTailMap = buf.validTailMap(-Double.MAX_VALUE);
                assertAll(
                        () -> assertEquals(2, recordTailMap.size()),
                        () -> assertEquals("r0 v2", recordTailMap.firstEntry().getValue().getValue()),
                        () -> assertEquals("r2 v1", recordTailMap.lastEntry().getValue().getValue()),
                        () -> assertEquals(2, validTailMap.size()),
                        () -> assertEquals("r2 v1", validTailMap.firstEntry().getValue().getValue()),
                        () -> assertEquals("r0 v2", validTailMap.lastEntry().getValue().getValue()));
            }
        }
    }
}
