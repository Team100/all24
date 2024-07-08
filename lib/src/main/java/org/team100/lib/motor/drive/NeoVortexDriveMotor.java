package org.team100.lib.motor.drive;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.model.NeoVortexTorqueModel;
import org.team100.lib.telemetry.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Linear drive motor using REV Neo Vortex */
public class NeoVortexDriveMotor extends CANSparkDriveMotor implements NeoVortexTorqueModel {

    public NeoVortexDriveMotor(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            // double gearRatio,
            // double wheelDiameter,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, new CANSparkFlex(canId, MotorType.kBrushless),
                motorPhase, currentLimit,
                //  gearRatio, wheelDiameter,
                ff, pid);
    }
}
