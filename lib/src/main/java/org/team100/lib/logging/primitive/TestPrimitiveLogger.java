package org.team100.lib.logging.primitive;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.team100.lib.util.Util;

/** Prints logs to stdout. */
public class TestPrimitiveLogger implements PrimitiveLogger {
    private final boolean m_print;
    private final Set<String> keys = new HashSet<>();

    public TestPrimitiveLogger(boolean print) {
        m_print = print;
    }

    public TestPrimitiveLogger() {
        this(false);
    }

    @Override
    public int keyCount() {
        return keys.size();
    }

    @Override
    public PrimitiveBooleanLogger booleanLogger(String label) {
        keys.add(label);
        return new PrimitiveBooleanLogger() {
            @Override
            public void log(boolean val) {
                if (m_print)
                    Util.printf("%s/%b\n", label, val);
            }
        };
    }

    @Override
    public PrimitiveDoubleLogger doubleLogger(String label) {
        keys.add(label);
        return new PrimitiveDoubleLogger() {
            @Override
            public void log(double val) {
                if (m_print)
                    Util.printf("%s/%.2f\n", label, val);
            }
        };
    }

    @Override
    public PrimitiveIntLogger intLogger(String label) {
        keys.add(label);
        return new PrimitiveIntLogger() {
            @Override
            public void log(int val) {
                if (m_print)
                    Util.printf("%s/%d\n", label, val);
            }
        };
    }

    @Override
    public PrimitiveDoubleArrayLogger doubleArrayLogger(String label) {
        keys.add(label);
        return new PrimitiveDoubleArrayLogger() {
            @Override
            public void log(double[] val) {
                if (m_print)
                    Util.printf("%s/%s\n", label, Arrays.toString(val));
            }
        };
    }

    @Override
    public PrimitiveLongLogger longLogger(String label) {
        keys.add(label);
        return new PrimitiveLongLogger() {
            @Override
            public void log(long val) {
                if (m_print)
                    Util.printf("%s/%d\n", label, val);
            }
        };
    }

    @Override
    public PrimitiveStringLogger stringLogger(String label) {
        keys.add(label);
        return new PrimitiveStringLogger() {
            @Override
            public void log(String val) {
                if (m_print)
                    Util.printf("%s/%s\n", label, val);
            }
        };
    }
}