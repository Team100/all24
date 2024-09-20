package org.team100.lib.logging;

/**
 * This should not be used by client code. Use {@link SupplierLogger} instead.
 * 
 * This is an abstract class so i can make these methods package-private.
 */
public abstract class PrimitiveLogger {

    abstract void logBoolean(String key, boolean val);

    abstract void logDouble(String key, double val);

    abstract void logInt(String key, int val);

    abstract void logDoubleArray(String key, double[] val);

    abstract void logDoubleObjArray(String key, Double[] val);

    abstract void logLong(String key, long val);

    abstract void logString(String key, String val);
}
