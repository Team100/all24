package org.team100.lib.async;

public interface Async {

    void addPeriodic(Runnable runnable, double periodS);
}
