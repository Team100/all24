package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.model.FalconTorqueModel;
import org.team100.lib.telemetry.Logger;

/** Falcon 500 using Phoenix 6. */
public class Falcon6Motor extends Talon6Motor implements FalconTorqueModel {

    public Falcon6Motor(
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
