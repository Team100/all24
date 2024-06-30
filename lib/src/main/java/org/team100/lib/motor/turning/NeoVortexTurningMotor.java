package org.team100.lib.motor.turning;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.model.NeoVortexTorqueModel;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * Swerve steering motor using REV Neo Vortex.
 */
public class NeoVortexTurningMotor extends CANSparkTurningMotor implements NeoVortexTorqueModel {

    public NeoVortexTurningMotor(
            String name,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            Feedforward100 ff,
            PIDConstants pid) {
        super(name, new CANSparkFlex(canId, MotorType.kBrushless),
                motorPhase, currentLimit, gearRatio, ff, pid);
    }

    @Override
    public String getGlassName() {
        return "NeoVortexTurningMotor";
    }
}