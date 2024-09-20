package org.team100.lib.logging;

import org.team100.lib.telemetry.Telemetry;

/** Doesn't do anything. */
public class TestLogger extends PrimitiveLogger {

    public SupplierLogger getSupplierLogger() {
        return new SupplierLogger(Telemetry.get(), "/",  this);
    }

    @Override
    public void logBoolean(String key, boolean val) {
    }

    @Override
    public void logDouble(String key, double vals) {
    }

    @Override
    public void logInt(String key, int vals) {
    }

    @Override
    public void logDoubleArray(String key, double[] val) {
    }

    @Override
    public void logDoubleObjArray(String key, Double[] val) {
    }

    @Override
    public void logLong(String key, long val) {
    }

    @Override
    public void logString(String key, String val) {
    }
}