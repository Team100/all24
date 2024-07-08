package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.model.KrakenTorqueModel;
import org.team100.lib.telemetry.Logger;

/** Kraken using Phoenix 6. */
public class Kraken6Motor extends Talon6Motor implements KrakenTorqueModel {

    public Kraken6Motor(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            PIDConstants pid,
            Feedforward100 ff) {
        super(parent, canId, motorPhase, supplyLimit, statorLimit, pid, ff);
    }
}
