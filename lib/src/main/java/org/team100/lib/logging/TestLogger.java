package org.team100.lib.logging;

import org.team100.lib.telemetry.Telemetry;

/** Doesn't do anything. */
public class TestLogger implements PrimitiveLogger2 {

    public SupplierLogger2 getSupplierLogger() {
        return new SupplierLogger2(Telemetry.get(), "/", this);
    }

    @Override
    public BooleanLogger booleanLogger(String label) {
        return new BooleanLogger() {
            @Override
            public void log(boolean val) {
            }
        };
    }

    @Override
    public DoubleLogger doubleLogger(String label) {
        return new DoubleLogger() {

            @Override
            public void log(double val) {
            }
        };
    }

    @Override
    public IntLogger intLogger(String label) {
        return new IntLogger() {
            @Override
            public void log(int val) {
            }
        };
    }

    @Override
    public DoubleArrayLogger doubleArrayLogger(String label) {
        return new DoubleArrayLogger() {
            @Override
            public void log(double[] val) {
            }
        };
    }

    @Override
    public DoubleObjArrayLogger doubleObjArrayLogger(String label) {
        return new DoubleObjArrayLogger() {
            @Override
            public void log(Double[] val) {
            }
        };
    }

    @Override
    public LongLogger longLogger(String label) {
        return new LongLogger() {
            @Override
            public void log(long val) {
            }
        };
    }

    @Override
    public StringLogger stringLogger(String label) {
        return new StringLogger() {
            @Override
            public void log(String val) {
            }
        };
    }
}