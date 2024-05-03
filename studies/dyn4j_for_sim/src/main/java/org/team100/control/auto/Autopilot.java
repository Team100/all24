package org.team100.control.auto;

import org.team100.control.Pilot;

/** Autopilot gets notified when commands are done. */
public interface Autopilot extends Pilot {
    void onEnd();
}
