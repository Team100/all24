package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.SupplierLogger2;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * Neo Vortex motor.
 * 
 * @see https://www.revrobotics.com/rev-21-1652/
 */
public class NeoVortexCANSparkMotor extends CANSparkMotor {
    public NeoVortexCANSparkMotor(
            SupplierLogger2 parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, new CANSparkFlex(canId, MotorType.kBrushless),
                motorPhase, currentLimit, ff, pid);
    }

    @Override
    public double kROhms() {
        return 0.057;
    }

    @Override
    public double kTNm_amp() {
        return 0.017;
    }
}
