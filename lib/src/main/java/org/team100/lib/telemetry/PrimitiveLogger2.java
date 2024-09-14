package org.team100.lib.telemetry;

/**
 * This should not be used by client code. Use {@link SupplierLogger} instead.
 */
public interface PrimitiveLogger2 {

    interface BooleanLogger {
        void log(boolean val);
    }

    BooleanLogger booleanLogger(String label);

    interface DoubleLogger {
        void log(double val);
    }

    DoubleLogger doubleLogger(String label);

    interface IntLogger {
        void log(int val);
    }

    IntLogger intLogger(String label);

    interface DoubleArrayLogger {
        void log(double[] val);
    }

    DoubleArrayLogger doubleArrayLogger(String label);

    interface DoubleObjArrayLogger {
        void log(Double[] val);
    }

    DoubleObjArrayLogger doubleObjArrayLogger(String label);

    interface LongLogger {
        void log(long val);
    }

    LongLogger longLogger(String label);

    interface StringLogger {
        void log(String val);
    }

    StringLogger stringLogger(String label);
}
