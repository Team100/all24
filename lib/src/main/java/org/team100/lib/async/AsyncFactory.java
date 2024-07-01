package org.team100.lib.async;

import org.team100.lib.framework.TimedRobot100;

public class AsyncFactory {
    // runner is made at startup so it's not possible to control with an
    // experiment. instead, these booleans will have to do.
    private static final boolean useTimedRobotAsync = true;
    private static final boolean useExecutorAsync = false;

    private final Async runner;

    public AsyncFactory(TimedRobot100 robot) {
        if (useTimedRobotAsync) {
            // Adds asyncs to the main loop callbacks.
            // This will slow down the main loop but avoid context-switching.
            runner = new TimedRobotAsync(robot);
        } else if (useExecutorAsync) {
            // Adds asyncs to a single-threaded java executor.
            // This avoids loading the main loop with the minimum number of threads.
            runner = new ExecutorAsync();
        } else {
            // Each async gets its own notifier thread.
            runner = new NotifierAsync();
        }
    }

    public Async get() {
        return runner;
    }

}
