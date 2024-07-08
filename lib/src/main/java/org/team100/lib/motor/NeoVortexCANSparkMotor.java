package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.model.NeoVortexTorqueModel;
import org.team100.lib.telemetry.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class NeoVortexCANSparkMotor extends CANSparkMotor implements NeoVortexTorqueModel {
    public NeoVortexCANSparkMotor(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, new CANSparkFlex(canId, MotorType.kBrushless),
                motorPhase, currentLimit, ff, pid);
    }
}
