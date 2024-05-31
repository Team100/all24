package org.team100.control.auto;

import org.team100.control.Pilot;

/**
 * Autonomous control is expressed through the Pilot interface, i.e. the
 * autonomous actor behaves like a human driver.
 * 
 * Autopilot gets notified when commands are done, so it can decide what to do
 * next.
 */
public interface Autopilot extends Pilot {
    void onEnd();
}
