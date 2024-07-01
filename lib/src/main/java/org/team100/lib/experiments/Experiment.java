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
    UseExecutorAsync,
    /**
     * Use Java executor for Command100.
     */
    UseCommandExecutor,
    /**
     * Periodically publish all tag poses to all cameras
     */
    UseCameraUpdater,
    /**
     * Filter rotational output to remove oscillation
     */
    UseThetaFilter,
    /**
     * Use outboard closed-loop position control for steering instead of onboard PID
     */
    OutboardSteering
}
