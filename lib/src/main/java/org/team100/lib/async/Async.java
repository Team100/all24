package org.team100.lib.async;

/** Various ways to execute asynchronous periodic functions. */
public interface Async {
    void addPeriodic(Runnable runnable, double periodS, String name);
}
