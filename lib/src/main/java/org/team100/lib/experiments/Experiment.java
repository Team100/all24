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
    OscillateDirect
}
