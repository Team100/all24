package org.team100.lib.encoder.turning;

/** Describes how the encoder angle is linked to the steering angle. */
public enum EncoderDrive {
    /**
     * Encoder moves the same as the module, e.g. via a belt, as in the WCP modules,
     * or via direct drive, as in the SDS modules, or via two gears in the AM
     * modules.
     */
    DIRECT,
    /**
     * Encoder moves opposite to the module, e.g. via a single gear, as in the
     * Team100 adaptation of the AM modules.
     */
    INVERSE
}