package org.team100.lib.telemetry;

/**
 * This should not be used by client code. Use {@link SupplierLogger} instead.
 * 
 * This is an abstract class so i can make these methods package-private.
 */
public abstract class PrimitiveLogger2 {

    abstract class BooleanLogger {
        abstract void log(boolean val);
    }

    /** Call once per key. */
    abstract BooleanLogger booleanLogger(String key);

    abstract void logDouble(String key, double val);

    abstract void logInt(String key, int val);

    abstract void logDoubleArray(String key, double[] val);

    abstract void logDoubleObjArray(String key, Double[] val);

    abstract void logLong(String key, long val);

    abstract class StringLogger {
        abstract void log(String val);
    }
}
