package org.team100.lib.motor.turning;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.model.NeoTorqueModel;
import org.team100.lib.telemetry.Telemetry.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/**
 * Swerve steering motor using REV Neo.
 */
public class NeoTurningMotor extends CANSparkTurningMotor implements NeoTorqueModel {
    public NeoTurningMotor(
            String name,
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            Feedforward100 ff,
            PIDConstants pid) {
        super(name, parent, new CANSparkMax(canId, MotorType.kBrushless),
                motorPhase, currentLimit, gearRatio, ff, pid);
    }
}