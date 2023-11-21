package org.team100.lib.motion.arm;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSubsystem extends Subsystem {
    public static class Config {
        public double filterTimeConstantS = 0.06;
        public double filterPeriodS = 0.02;
    }

    private final Config m_config = new Config();
    private final Telemetry t = Telemetry.get();
    private final LinearFilter m_lowerMeasurementFilter;
    private final LinearFilter m_upperMeasurementFilter;
    private final CANSparkMax lowerArmMotor;
    private final CANSparkMax upperArmMotor;
    private final AnalogInput lowerArmInput;
    private final AnalogInput upperArmInput;
    private final AnalogEncoder lowerArmEncoder;
    private final AnalogEncoder upperArmEncoder;
    private ArmAngles previousPosition;

    public ArmSubsystem() {
        m_lowerMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        m_upperMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);

        lowerArmMotor = new CANSparkMax(4, MotorType.kBrushless);
        lowerArmMotor.restoreFactoryDefaults();
        lowerArmMotor.setSmartCurrentLimit(8);
        lowerArmMotor.setSecondaryCurrentLimit(8);
        lowerArmMotor.setIdleMode(IdleMode.kBrake);

        upperArmMotor = new CANSparkMax(30, MotorType.kBrushless);
        upperArmMotor.restoreFactoryDefaults();
        upperArmMotor.setSmartCurrentLimit(1);
        upperArmMotor.setSecondaryCurrentLimit(1);
        upperArmMotor.setIdleMode(IdleMode.kBrake);

        lowerArmInput = new AnalogInput(1);
        lowerArmEncoder = new AnalogEncoder(lowerArmInput);
        upperArmInput = new AnalogInput(0);
        upperArmEncoder = new AnalogEncoder(upperArmInput);
        previousPosition = getPosition();
    }

    /** Arm angles (radians), 0 up, positive forward. */
    public ArmAngles getPosition() {
        ArmAngles result = new ArmAngles(
                m_lowerMeasurementFilter.calculate(getLowerArm()),
                m_upperMeasurementFilter.calculate(getUpperArmAngleRadians()));
        t.log(Level.DEBUG, "/arm/Lower Encoder Pos: ", result.th1);
        t.log(Level.DEBUG, "/arm/Upper Encoder Pos: ", result.th2);
        return result;
    }

    /** Joint velocities in radians per second. */
    public ArmAngles getVelocity() {
        ArmAngles position = getPosition();
        double th1 = position.th1 - previousPosition.th1;
        double th2 = position.th2 - previousPosition.th2;
        previousPosition = position;
        ArmAngles result = new ArmAngles(th1 * 50, th2 * 50);
        t.log(Level.DEBUG, "/arm/Lower Encoder Vel: ", result.th1);
        t.log(Level.DEBUG, "/arm/Upper Encoder Vel: ", result.th2);
        return result;
    }

    public void set(double u1, double u2) {
        lowerArmMotor.set(u1);
        upperArmMotor.set(u2);
    }

    /** Lower arm angle (radians), 0 up, positive forward. */
    private double getLowerArm() {
        double encoderZero = 0.861614;
        double x = (lowerArmEncoder.getAbsolutePosition() - encoderZero) * 360;
        return (-1.0 * x) * Math.PI / 180;
    }

    /** Upper arm angle (radians), 0 up, positive forward. */
    private double getUpperArmAngleRadians() {
        double encoderZero = 0.266396;
        double x = (upperArmEncoder.getAbsolutePosition() - encoderZero) * 360;
        return x * Math.PI / 180;
    }

}
