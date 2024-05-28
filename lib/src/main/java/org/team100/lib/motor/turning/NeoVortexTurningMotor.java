package org.team100.lib.motor.turning;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * Swerve steering motor using REV Neo Vortex.
 */
public class NeoVortexTurningMotor extends CANSparkTurningMotor {

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
    double kROhms() {
        // @see https://www.revrobotics.com/rev-21-1652/
        return 0.057;
    }

    @Override
    double kTNm_amp() {
        // @see https://www.revrobotics.com/rev-21-1652/
        return 0.017;
    }
}