package org.team100.lib.async;

public class MockAsync implements Async {

    @Override
    public void addPeriodic(Runnable runnable, double periodS, String name) {
        // doesn't do anything
    }
    
}
