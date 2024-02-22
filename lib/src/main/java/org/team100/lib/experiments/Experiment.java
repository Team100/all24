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
    HeedVision
}
