package org.team100.lib.motor.drive;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.model.KrakenTorqueModel;
import org.team100.lib.telemetry.Telemetry.Logger;

/** Kraken using Phoenix 6. */
public class Kraken6DriveMotor extends Talon6DriveMotor implements KrakenTorqueModel {

    public Kraken6DriveMotor(
            String name,
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            double kDriveReduction,
            double wheelDiameter,
            PIDConstants pid,
            Feedforward100 ff) {
        super(name, parent, canId, motorPhase, supplyLimit,
                statorLimit, kDriveReduction, wheelDiameter,
                pid, ff);
    }
}
