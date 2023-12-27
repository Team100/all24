package org.team100.lib.experiments;

/**
 * An experiment is something that can be selectively enabled.
 */
public enum Experiment {
    /** Smooth chassis speeds. */
    UseSetpointGenerator,
    /** Offload the drive PID to the motor controller. */
    UseClosedLoopDrive,
    /** Offload the steering PID to the motor controller. */
    UseClosedLoopSteering,
    /** Offload simple servo velocity to the controller */
    UseClosedLoopVelocity,
    /** Use initial state in trajectory generation */
    UseInitialVelocity,
    /** For the oscillate command, drive modules directly */
    OscillateDirect,
    /**
     * Flush network tables as often as possible. Do not enable this experiment in
     * competition, you'll overwhelm the network and the RIO
     */
    FlushOften,
    /**
     * Show mode locks slow speed for younger drivers.
     */
    ShowMode
}
