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
     * Use initial state in trajectory generation
     */
    UseInitialVelocity,
    /**
     * For the oscillate command, drive modules directly
     */
    OscillateDirect,
    /**
     * For the oscillate command, drive rotation instead of linearly
     */
    OscillateTheta,
    /**
     * Flush network tables as often as possible. Do not enable this experiment in
     * competition, you'll overwhelm the network and the RIO
     */
    FlushOften,
    /**
     * Use multiple april tags to triangulate position
     */
    Triangulate,
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
     * Use note detection in auto
     */
    AutoNoteDetection,
    /**
     * Use full state feedback for trajectory following
     */
    FullStateTrajectoryFollower,
    /**
     * Compensate for drive/steer coupling
     * https://docs.google.com/document/d/1Zm6VpteqNMmT0VaTDhN5U6-jF3VS11uCoykzZUIGQdU/edit
     */
    DriveSteerCouplingCompensation,
    /**
     * Use softer vision update gains
     */
    AvoidVisionJitter,
    /**
     * Include sag limiter in setpoint generator
     */
    LimitBatterySag,
    /**
     * Use slippery tire model
     */
    SlipperyTires,
    /**
     * Use executor service for asyncs
     */
    UseExecutorAsync
}
