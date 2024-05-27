package org.team100.lib.motor;

import java.util.function.Supplier;

import org.team100.lib.config.PIDConstants;
import org.team100.lib.util.Util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

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

    public static void motorConfig(CANSparkBase motor, IdleMode idleMode, MotorPhase phase, int velocityMeasurementPeriod) {
        crash(() -> motor.setIdleMode(idleMode));
        motor.setInverted(phase == MotorPhase.REVERSE);
        // velocity is in the Status1 frame
        // position is in the Status2 frame.
        // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        crash(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, velocityMeasurementPeriod));
        crash(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, velocityMeasurementPeriod));
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
