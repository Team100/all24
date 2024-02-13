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
     * Offload simple servo velocity to the controller
     */
    UseClosedLoopVelocity,
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
     * Show mode locks slow speed for younger drivers.
     */
    ShowMode,
    /**
     * Use multiple april tags to triangulate position
     */
    Triangulate
}
