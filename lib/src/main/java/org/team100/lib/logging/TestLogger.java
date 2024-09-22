package org.team100.lib.logging;

import org.team100.lib.telemetry.Telemetry.Level;

/** Doesn't do anything. */
public class TestLogger implements PrimitiveLogger2 {

    public SupplierLogger2 getSupplierLogger() {
        return new SupplierLogger2(() -> Level.TRACE, "test", this);
    }

    @Override
    public BooleanLogger booleanLogger(String label) {
        return x -> {
        };
    }

    @Override
    public DoubleLogger doubleLogger(String label) {
        return x -> {
        };
    }

    @Override
    public IntLogger intLogger(String label) {
        return x -> {
        };
    }

    @Override
    public DoubleArrayLogger doubleArrayLogger(String label) {
        return x -> {
        };
    }

    @Override
    public DoubleObjArrayLogger doubleObjArrayLogger(String label) {
        return x -> {
        };
    }

    @Override
    public LongLogger longLogger(String label) {
        return x -> {
        };
    }

    @Override
    public StringLogger stringLogger(String label) {
        return x -> {
        };
    }
}