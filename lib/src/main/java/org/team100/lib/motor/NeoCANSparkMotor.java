package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.SupplierLogger2;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/**
 * Neo motor.
 * 
 * @see https://www.revrobotics.com/rev-21-1650/
 */
public class NeoCANSparkMotor extends CANSparkMotor {
    /**
     * Note the PID values should be in duty cycle per RPM, i.e. very small. {@link
     * Rev100}
     */
    public NeoCANSparkMotor(
            SupplierLogger2 parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, new CANSparkMax(canId, MotorType.kBrushless),
                motorPhase, currentLimit, ff, pid);
    }

    @Override
    public double kROhms() {
        return 0.114;
    }

    @Override
    public double kTNm_amp() {
        return 0.028;
    }
}
