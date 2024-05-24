package org.team100.lib.async;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

public class AsyncFactory {
    private static final Async runner;
    static {
        if (Experiments.instance.enabled(Experiment.UseExecutorAsync)) {
            runner = new ExecutorAsync();
        } else {
            runner = new NotifierAsync();
        }
    }

    public static Async get() {
        return runner;
    }

    private AsyncFactory() {
        //
    }

}
