package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.model.NeoTorqueModel;
import org.team100.lib.telemetry.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class NeoCANSparkMotor extends CANSparkMotor implements NeoTorqueModel {
    public NeoCANSparkMotor(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, new CANSparkMax(canId, MotorType.kBrushless),
                motorPhase, currentLimit, ff, pid);
    }
}
