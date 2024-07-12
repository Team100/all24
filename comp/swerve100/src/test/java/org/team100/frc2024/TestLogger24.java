package org.team100.frc2024;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.util.function.FloatSupplier;

public class TestLogger24 implements Logger {
    @Override
    public Logger child(String stem) {
        return this;
    }

    @Override
    public void logBoolean(Level level, String leaf, BooleanSupplier val) {
    }

    @Override
    public void logDouble(Level level, String leaf, DoubleSupplier vals) {
    }

    @Override
    public void logInt(Level level, String leaf, IntSupplier vals) {
    }

    @Override
    public void logFloat(Level level, String leaf, FloatSupplier val) {
    }

    @Override
    public void logDoubleArray(Level level, String leaf, Supplier<double[]> val) {
    }

    @Override
    public void logDoubleObjArray(Level level, String leaf, Supplier<Double[]> val) {
    }

    @Override
    public void logLong(Level level, String leaf, LongSupplier val) {
    }

    @Override
    public void logString(Level level, String leaf, Supplier<String> val) {
    }

    @Override
    public void logStringArray(Level level, String leaf, Supplier<String[]> val) {
    }
}
