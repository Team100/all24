package org.team100.lib.motor.drive;

import org.team100.lib.telemetry.Telemetry;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.MathUtil;

/**
 * Uses default position/velocity sensor which is the integrated one.
 * 
 * See details on velocity averaging and sampling.
 * https://v5.docs.ctr-electronics.com/en/stable/ch14_MCSensor.html#velocity-measurement-filter
 * 
 * Summarized:
 * 
 * * every 1ms, the velocity is measured as (x(t) - x(t-p)) / p
 * * N such measurements are averaged
 * 
 * Default p is 100 ms, default N is 64, so, if the velocity measurement is
 * perfect, it represents the actual velocity 82 ms ago, which is a long time.
 * 
 * For awhile we had p set to 0.001 and N set to 1. It's kind of amazing that it
 * worked at all. Full speed is roughly 6k rpm or 100 rev/s or 0.1 rev per
 * sample or 200 ticks per sample, so that's fine. But moving slowly, say 0.1
 * m/s, that's 2 ticks per sample, which means that the velocity controller is
 * going to see a lot of quantization noise in the measurement.
 * 
 * A better setting would use a window that's about the width of the control
 * period, 20ms, so try 10ms and 8 samples. Note, this delay will be noticeable.
 * 
 * TODO: deal with delay in velocity measurement.
 */
public class FalconDriveMotor implements DriveMotor {
    private static final double ticksPerRevolution = 2048;

    private final Telemetry t = Telemetry.get();
    private final WPI_TalonFX m_motor;
    private final double m_gearRatio;
    private final double m_wheelDiameter;
    private final String m_name;

    /**
     * Throws if any of the configurations fail.
     */
    public FalconDriveMotor(String name, int canId, double currentLimit, double kDriveReduction, double wheelDiameter) {
        m_wheelDiameter = wheelDiameter;
        m_gearRatio = kDriveReduction;
        m_motor = new WPI_TalonFX(canId);
        m_motor.configFactoryDefault();
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        m_motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimit, 0));
        m_motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 0));
        m_motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
        m_motor.configVelocityMeasurementWindow(8);
        m_motor.configVoltageCompSaturation(11);
        m_motor.enableVoltageCompensation(true);
        m_motor.configNominalOutputForward(0);
        m_motor.configNominalOutputReverse(0);
        m_motor.configPeakOutputForward(1);
        m_motor.configPeakOutputReverse(-1);
        m_motor.config_kF(0, 0);
        m_motor.config_kP(0, 0.05);
        m_motor.config_kI(0, 0);
        m_motor.config_kD(0, 0);

        m_name = String.format("/Falcon Drive Motor %s", name);
        t.log(m_name + "/Device ID", m_motor.getDeviceID());
    }

    private void require(ErrorCode errorCode) {
        if (errorCode != ErrorCode.OK)
            throw new IllegalArgumentException("motor configuration error: " + errorCode.name());
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    @Override
    public void set(double output) {
        m_motor.setVoltage(10 * MathUtil.clamp(output, -1.3, 1.3));

        t.log(m_name + "/Speed (rot_s)", getVelocityRot_S());
        t.log(m_name + "/Output [-1,1]", get());
        t.log(m_name + "/Speed (2048ths_100ms)", getVelocity2048_100());
    }

    @Override
    public void setPID(ControlMode control, double outputMetersPerSec) {
        double Kn = 0.11106;
        double Ks = 0.001515;
        double VSat = 11;
        double revolutionsPerSec = outputMetersPerSec / (m_wheelDiameter * Math.PI);
        double revsPer100ms = revolutionsPerSec / 10;
        double ticksPer100ms = revsPer100ms * ticksPerRevolution;
        if (revolutionsPerSec < .575) {
            Ks = .03;
        }
        double kFF = (Kn * revolutionsPerSec + Ks * Math.signum(revolutionsPerSec)) * m_gearRatio / VSat;
        DemandType type = DemandType.ArbitraryFeedForward;
        m_motor.set(ControlMode.Velocity, ticksPer100ms * m_gearRatio, type, kFF);

        t.log(m_name + "/Speed (rot_s)", getVelocityRot_S());
        t.log(m_name + "/Output [-1,1]", get());
        t.log(m_name + "/Speed (2048ths_100ms)", getVelocity2048_100());
    }

    double getVelocityRot_S() {
        return m_motor.getSelectedSensorVelocity() / (ticksPerRevolution / 10 * m_gearRatio);
    }

    /**
     * @return integrated sensor position in sensor units (1/2048 turn).
     */
    public double getPosition() {
        return m_motor.getSelectedSensorPosition();
    }

    /**
     * @return integrated sensor velocity in sensor units (1/2048 turn) per 100ms.
     */
    public double getVelocity2048_100() {
        return m_motor.getSelectedSensorVelocity();
    }

    /**
     * Sets integrated sensor position to zero.
     * Throws if it fails, watch out, don't use this during a match.
     */
    public void resetPosition() {
        require(m_motor.setSelectedSensorPosition(0));
    }
}
