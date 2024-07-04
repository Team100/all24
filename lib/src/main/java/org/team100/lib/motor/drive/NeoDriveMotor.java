package org.team100.lib.motor.drive;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.model.NeoTorqueModel;
import org.team100.lib.telemetry.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/** Linear drive motor using REV Neo. */
public class NeoDriveMotor extends CANSparkDriveMotor implements NeoTorqueModel {

    public NeoDriveMotor(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            double wheelDiameter,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, new CANSparkMax(canId, MotorType.kBrushless),
                motorPhase, currentLimit, gearRatio, wheelDiameter,
                ff, pid);
    }
}
