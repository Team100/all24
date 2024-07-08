package org.team100.lib.telemetry;

/** Doesn't do anything, always disabled. */
public class TestLogger implements Logger {
    @Override
    public Logger child(String stem) {
        return this;
    }
}