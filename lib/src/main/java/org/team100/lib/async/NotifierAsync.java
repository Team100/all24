package org.team100.lib.async;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;

/**
 * Support async periodic execution with a notifier thread per task.
 * 
 * Note using a thread per task can result in a lot of threads, which might not
 * be great.
 */
public class NotifierAsync implements Async {
    private final List<Notifier> m_notifiers;

    NotifierAsync() {
        m_notifiers = new ArrayList<>();
    }

    @Override
    public void addPeriodic(Runnable runnable, double periodS, String name) {
        Notifier n = new Notifier(runnable);
        n.setName(name);
        n.startPeriodic(periodS);
        m_notifiers.add(n);
    }
}
