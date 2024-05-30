package org.team100.lib.motor.turning;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.model.FalconTorqueModel;

public class Falcon6TurningMotor extends Talon6TurningMotor implements FalconTorqueModel {

    public Falcon6TurningMotor(
            String name,
            int canId,
            MotorPhase motorPhase,
            double kGearRatio,
            PIDConstants pid,
            Feedforward100 ff) {
        super(name, canId, motorPhase, kGearRatio, pid, ff);
    }
}
