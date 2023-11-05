package org.team100.lib.storage;

import java.util.Map.Entry;
import java.util.NavigableMap;

/** History that can be trimmed. */
public class EditableHistory<Value> extends History<Value> {
    private static final boolean debug = false;

    public EditableHistory(int capacity) {
        super(capacity);
    }

    /** remove history starting from vt */
    public void trim(double vt) {
        NavigableMap<Double, Value> tailMap = mutableValidTailMap(vt);
        if (debug) {
            System.out.println("trim " + vt);
            System.out.println("tailmap size " + tailMap.size());
            for (Entry<Double, Value> e : tailMap.entrySet()) {
                System.out.println("trimming " + e.getKey() + " " + e.getValue());
            }
        }
        tailMap.clear();
    }
}
