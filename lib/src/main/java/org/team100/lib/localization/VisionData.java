package org.team100.lib.localization;

public interface VisionData {
    /**
     * Read queued network input, if any, and distribute it to the pose
     * estimator and firing solution consumers.
     */
    void update();
}
