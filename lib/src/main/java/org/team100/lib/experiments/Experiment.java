package org.team100.lib.experiments;

/**
 * An experiment is something that can be selectively enabled.
 */
public enum Experiment {
    /**
     * Smooth chassis speeds.
     */
    UseSetpointGenerator,
    /**
     * Flush network tables as often as possible. Do not enable this experiment in
     * competition, you'll overwhelm the network and the RIO
     */
    FlushOften,
    /**
     * Pay attention to camera input. It's useful to turn this off for testing and
     * calibration.
     */
    HeedVision,
    /**
     * Control heading when manually steering in snaps mode, to prevent
     * rotational drifting.
     */
    StickyHeading,
    /**
     * Drive to note turns toward the note
     */
    DriveToNoteWithRotation,
    /**
     * Use full state feedback for trajectory following
     */
    FullStateTrajectoryFollower,
    /**
     * Use softer vision update gains
     */
    AvoidVisionJitter,
    /**
     * Filter snap rotational output to remove oscillation
     */
    SnapThetaFilter,
    /**
     * Use low-pass filter and deadbanding on controller feedback, to prevent
     * overresponse to noise and jitter around zero.
     */
    FilterFeedback,
    /**
     * Use the network-tables gyro. 
     */
    NetworkGyro,
    /**
     * Snaps can prefer rotation or translation
     */
    SnapPreferRotation,
    /**
     * Clip the snap omega
     */
    SnapGentle
}
