package org.team100.lib.logging.primitive;

/**
 * This should not be used by client code. Use {@link LoggerFactory} instead.
 */
public interface PrimitiveLogger {

    int keyCount();

    @FunctionalInterface
    interface PrimitiveBooleanLogger {
        void log(boolean val);
    }

    PrimitiveBooleanLogger booleanLogger(String label);

    @FunctionalInterface
    interface PrimitiveDoubleLogger {
        void log(double val);
    }

    PrimitiveDoubleLogger doubleLogger(String label);

    @FunctionalInterface
    interface PrimitiveIntLogger {
        void log(int val);
    }

    PrimitiveIntLogger intLogger(String label);

    @FunctionalInterface
    interface PrimitiveDoubleArrayLogger {
        void log(double[] val);
    }

    PrimitiveDoubleArrayLogger doubleArrayLogger(String label);

    @FunctionalInterface
    interface PrimitiveLongLogger {
        void log(long val);
    }

    PrimitiveLongLogger longLogger(String label);

    @FunctionalInterface
    interface PrimitiveStringLogger {
        void log(String val);
    }

    PrimitiveStringLogger stringLogger(String label);
}
