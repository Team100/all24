package org.team100.lib.motor.drive;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.model.FalconTorqueModel;
import org.team100.lib.telemetry.Logger;

/** Falcon 500 using Phoenix 6. */
public class Falcon6DriveMotor extends Talon6DriveMotor implements FalconTorqueModel {

    public Falcon6DriveMotor(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            double kDriveReduction,
            double wheelDiameter,
            PIDConstants pid,
            Feedforward100 ff) {
        super(parent, canId, motorPhase, supplyLimit,
                statorLimit, kDriveReduction, wheelDiameter,
                pid, ff);
    }
}
