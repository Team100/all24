package org.team100.lib.async;

import org.team100.lib.framework.TimedRobot100;

public class AsyncFactory {
    // runner is made at startup so it's not possible to control with an
    // experiment. instead, this enum will have to do.
    private static final AsyncType TYPE = AsyncType.TIMED;

    private enum AsyncType {
        TIMED,
        EXECUTOR,
        NOTIFIER
    }

    private final Async runner;

    public AsyncFactory(TimedRobot100 robot) {
        runner = switch (TYPE) {
            // Adds asyncs to the main loop callbacks.
            // This will slow down the main loop but avoid context-switching.
            case TIMED -> new TimedRobotAsync(robot);
            // Adds asyncs to a single-threaded java executor.
            // This avoids loading the main loop with the minimum number of threads.
            case EXECUTOR -> new ExecutorAsync();
            // Each async gets its own notifier thread.
            case NOTIFIER -> new NotifierAsync();
        };

    }

    public Async get() {
        return runner;
    }

}
