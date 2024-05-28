package org.team100.lib.motor.drive;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/**
 * Linear drive motor using REV Neo.
 */
public class NeoDriveMotor extends CANSparkDriveMotor {

    @Override
    double kROhms() {
        // @see https://www.revrobotics.com/rev-21-1650/
        return 0.114;
    }

    @Override
    double kTNm_amp() {
        // @see https://www.revrobotics.com/rev-21-1650/
        return 0.028;
    }

    public NeoDriveMotor(
            String name,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            double wheelDiameter,
            Feedforward100 ff,
            PIDConstants pid) {
        super(name, new CANSparkMax(canId, MotorType.kBrushless),
                motorPhase, currentLimit, gearRatio, wheelDiameter,
                ff, pid);
    }
}
