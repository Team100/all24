package org.team100.lib.motor;

import java.util.function.Supplier;

import org.team100.lib.config.PIDConstants;
import org.team100.lib.util.Util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;

public class Rev100 {

    public static void crash(Supplier<REVLibError> s) {
        REVLibError errorCode = s.get();
        if (errorCode != REVLibError.kOk) {
            throw new IllegalStateException(errorCode.name());
        }
    }

    public static void warn(Supplier<REVLibError> s) {
        REVLibError errorCode = s.get();
        if (errorCode != REVLibError.kOk) {
            Util.warn(errorCode.name());
        }
    }

    public static void baseConfig(CANSparkBase motor) {
        crash(motor::restoreFactoryDefaults);
    }

    public static void motorConfig(CANSparkBase motor) {
        crash(() -> motor.setIdleMode(IdleMode.kBrake));
    }

    public static void currentConfig(CANSparkBase motor, int currentLimit) {
        crash(() -> motor.setSmartCurrentLimit(currentLimit));
    }

    public static void pidConfig(SparkPIDController control, PIDConstants pid) {
        crash(() -> control.setPositionPIDWrappingEnabled(false)); // don't use position control
        crash(() -> control.setP(pid.getP()));
        crash(() -> control.setI(pid.getI()));
        crash(() -> control.setD(pid.getD()));
        crash(() -> control.setIZone(pid.getIZone()));
        crash(() -> control.setFF(0)); // use arbitrary FF instead
        crash(() -> control.setOutputRange(-1, 1));
    }

    private Rev100() {
        //
    }

}
