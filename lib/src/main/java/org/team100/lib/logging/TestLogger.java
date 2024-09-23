package org.team100.lib.logging;

import java.util.Arrays;

import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

/** Prints logs to stdout. */
public class TestLogger implements PrimitiveLogger2 {
    private final boolean m_print;

    public TestLogger(boolean print) {
        m_print = print;
    }

    public TestLogger() {
        this(false);
    }

    public SupplierLogger2 getSupplierLogger() {
        return new SupplierLogger2(() -> Level.TRACE, "test", this);
    }

    @Override
    public BooleanLogger booleanLogger(String label) {
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
        return new StringLogger() {
            @Override
            public void log(String val) {
                if (m_print)
                    Util.printf("%s/%s\n", label, val);
            }
        };
    }
}