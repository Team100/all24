package org.team100.lib.async;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;

/** Support async periodic execution with a notifier. */
public class NotifierAsync implements Async {
    private final List<Notifier> m_notifiers;

    NotifierAsync() {
        m_notifiers = new ArrayList<>();
    }

    @Override
    public void addPeriodic(Runnable runnable, double periodS) {
        Notifier n = new Notifier(runnable);
        n.startPeriodic(periodS);
        m_notifiers.add(n);
    }
}
