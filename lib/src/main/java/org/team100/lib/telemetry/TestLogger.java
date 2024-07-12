package org.team100.lib.telemetry;

/** Doesn't do anything. */
public class TestLogger extends PrimitiveLogger {

    public SupplierLogger getSupplierLogger() {
        return new SupplierLogger(Telemetry.get(), "/", () -> false, this, () -> false, this);
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
    public void logFloat(String key, float val) {
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

    @Override
    public void logStringArray(String key, String[] val) {
    }
}