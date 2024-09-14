package org.team100.lib.telemetry;

/**
 * This should not be used by client code. Use {@link SupplierLogger} instead.
 * 
 * This is an abstract class so i can make these methods package-private.
 */
public interface PrimitiveLogger2 {

    interface BooleanLogger {
        void log(boolean val);
    }

    interface DoubleLogger {
        void log(double val);
    }

    interface IntLogger {
        void log(int val);
    }

    interface DoubleArrayLogger {
        void log(double[] val);
    }

    interface DoubleObjArrayLogger {
        void log(Double[] val);
    }

    interface LongLogger {
        void log(long val);
    }

    interface StringLogger {
        void log(String val);
    }
}
