package org.team100.lib.logging;

/**
 * This should not be used by client code. Use {@link SupplierLogger} instead.
 */
public interface PrimitiveLogger2 {

    @FunctionalInterface
    interface BooleanLogger {
        void log(boolean val);
    }

    BooleanLogger booleanLogger(String label);

    @FunctionalInterface
    interface DoubleLogger {
        void log(double val);
    }

    DoubleLogger doubleLogger(String label);

    @FunctionalInterface
    interface IntLogger {
        void log(int val);
    }

    IntLogger intLogger(String label);

    @FunctionalInterface
    interface DoubleArrayLogger {
        void log(double[] val);
    }

    DoubleArrayLogger doubleArrayLogger(String label);

    @FunctionalInterface
    interface DoubleObjArrayLogger {
        void log(Double[] val);
    }

    DoubleObjArrayLogger doubleObjArrayLogger(String label);

    @FunctionalInterface
    interface LongLogger {
        void log(long val);
    }

    LongLogger longLogger(String label);

    @FunctionalInterface
    interface StringLogger {
        void log(String val);
    }

    StringLogger stringLogger(String label);
}
