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
    public BooleanLogger booleanLogger(String label) {
        keys.add(label);
        return new BooleanLogger() {
            @Override
            public void log(boolean val) {
                if (m_print)
                    Util.printf("%s/%b\n", label, val);
            }
        };
    }

    @Override
    public DoubleLogger doubleLogger(String label) {
        keys.add(label);
        return new DoubleLogger() {
            @Override
            public void log(double val) {
                if (m_print)
                    Util.printf("%s/%.2f\n", label, val);
            }
        };
    }

    @Override
    public IntLogger intLogger(String label) {
        keys.add(label);
        return new IntLogger() {
            @Override
            public void log(int val) {
                if (m_print)
                    Util.printf("%s/%d\n", label, val);
            }
        };
    }

    @Override
    public DoubleArrayLogger doubleArrayLogger(String label) {
        keys.add(label);
        return new DoubleArrayLogger() {
            @Override
            public void log(double[] val) {
                if (m_print)
                    Util.printf("%s/%s\n", label, Arrays.toString(val));
            }
        };
    }

    @Override
    public DoubleObjArrayLogger doubleObjArrayLogger(String label) {
        keys.add(label);
        return new DoubleObjArrayLogger() {
            @Override
            public void log(Double[] val) {
                if (m_print)
                    Util.printf("%s/%s\n", label, Arrays.toString(val));
            }
        };
    }

    @Override
    public LongLogger longLogger(String label) {
        keys.add(label);
        return new LongLogger() {
            @Override
            public void log(long val) {
                if (m_print)
                    Util.printf("%s/%d\n", label, val);
            }
        };
    }

    @Override
    public StringLogger stringLogger(String label) {
        keys.add(label);
        return new StringLogger() {
            @Override
            public void log(String val) {
                if (m_print)
                    Util.printf("%s/%s\n", label, val);
            }
        };
    }
}