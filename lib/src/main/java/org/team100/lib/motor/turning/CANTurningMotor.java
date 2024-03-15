package org.team100.lib.motor.turning;

import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/**
 * Swerve steering motor using TalonSRX, which is used with the 775, using a
 * quadrature encoder, in the andymark swerves.
 * 
 * https://www.andymark.com/products/hex-pg-series-gearboxes-options
 * 
 * Because the number of ticks per revolution is small, the velocity measurement
 * is coarse at low speed.
 *
 * One motor tick is 0.22 rad, measuring across 50ms means we'd like to have a
 * minimum tick rate of 20hz, so 4.4 rad/s.
 * 
 * Through the gearbox, that/s 0.074 rad/s, or about 4 degrees/sec.
 * 
 * There are higher-resolution encoders available, which would solve the
 * problem, though the cost might not be worth the effort.
 * 
 * https://www.andymark.com/products/am-mag-encoder
 * 
 * Given the issues with feedback, this controller should rely mostly on
 * feedforward.
 */
public class CANTurningMotor implements Motor100<Angle100> {
    /**
     * There is a planetary gearbox between the motor and the steering gear, and the
     * final is 48/40.
     */
    private static final double m_gearRatio = 71.0 * 40 / 48;
    /**
     * The encoder is a quadrature encoder with 7 cycles per revolution, so 28
     * countable events.
     */
    private static final double ticksPerRevolution = 28;
    private static final double kTurningCurrentLimit = 7;

    /**
     * Friction feedforward in volts, for when the mechanism is moving.
     * 
     * This value is pretty high, due to the friction in the gearbox.
     */
    private static final double dynamicFrictionFFVolts = 0.448;

    /**
     * Velocity feedforward in units of volts per motor revolution per second, or
     * volt-seconds per revolution.
     * 
     * I think this value is probably wrong. The free speed of a 775 is about
     * 18krpm, so this should really be about 0.036. How did this get here?
     */
    private static final double velocityFFVoltS_Rev = 0.11;

    /**
     * Placeholder for accel feedforward.
     */
    private static final double accelFFVoltS2_M = 0;

    /**
     * Proportional feedback coefficient for the controller. The error is measured
     * in sensor units (ticks per 100ms), and the full scale output is 1023.
     */
    private static final double outboardP = 0.5;

    /**
     * For voltage compensation, the maximum output voltage.
     */
    private static final double saturationVoltage = 11;

    private final Telemetry t = Telemetry.get();
    private final WPI_TalonSRX m_motor;
    private final String m_name;

    public CANTurningMotor(String name, int channel) {
        m_motor = new WPI_TalonSRX(channel);
        m_motor.configFactoryDefault();
        m_motor.setNeutralMode(NeutralMode.Brake);

        // reset position to zero (position is not used, this is just for logging)
        m_motor.setSelectedSensorPosition(0);

        m_motor.enableCurrentLimit(true);
        m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, kTurningCurrentLimit, kTurningCurrentLimit, 0));

        // use the quadrature encoder for status and PID feedback
        m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // configure velocity measurement sampling
        // note this is quite a long period because the encoder resolution is low.
        m_motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);
        m_motor.configVelocityMeasurementWindow(8);

        // configure voltage compensation
        m_motor.configVoltageCompSaturation(saturationVoltage);
        m_motor.enableVoltageCompensation(true);

        // configure output limits
        m_motor.configNominalOutputForward(0);
        m_motor.configNominalOutputReverse(0);
        m_motor.configPeakOutputForward(1);
        m_motor.configPeakOutputReverse(-1);

        // configure outboard PID
        m_motor.config_kP(0, outboardP);
        m_motor.config_kI(0, 0);
        m_motor.config_kD(0, 0);
        m_motor.config_kF(0, 0);

        m_motor.setSensorPhase(true);

        m_name = Names.append(name, this);
        t.log(Level.TRACE, m_name, "Device ID", m_motor.getDeviceID());
    }

    public WPI_TalonSRX getMotor() {
        return m_motor;
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.log(Level.TRACE, m_name, "Output", output);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    /**
     * Supports accel feedforward.
     */
    @Override
    public void setVelocity(double outputRad_S, double accelRad_S2) {
        double outputRev_S = outputRad_S / (2 * Math.PI);
        double motorRev_S = outputRev_S * m_gearRatio;
        double motorRev_100ms = motorRev_S / 10;
        double motorTick_100ms = motorRev_100ms * ticksPerRevolution;

        double velocityFF = velocityFF(motorRev_S);
        double frictionFF = frictionFF(motorRev_S);
        double accelFF = accelFF(accelRad_S2);
        
        double kFF = frictionFF + velocityFF + accelFF;

        m_motor.set(ControlMode.Velocity, motorTick_100ms, DemandType.ArbitraryFeedForward, kFF);
    }

    /**
     * ignores torque feedforward
     */
    @Override
    public void setVelocity(double velocity, double accel, double torque) {
        setVelocity(torque, accel);
    }

    @Override
    public double getTorque() {
        throw new UnsupportedOperationException("Unimplemented method 'getTorque'");
    }

    @Override
    public void close() {
        m_motor.close();
    }

    @Override
    public void periodic() {
        t.log(Level.TRACE, m_name, "Encoder Value",
                m_motor.getSelectedSensorPosition() / (m_gearRatio * ticksPerRevolution));
        t.log(Level.TRACE, m_name, "Velocity Value",
                m_motor.getSelectedSensorVelocity() / (ticksPerRevolution * m_gearRatio) * 10);
    }

    ///////////////////////////////////////////////////////

    /**
     * Frictional feedforward in duty cycle units [-1, 1]
     */
    private static double frictionFF(double motorRev_S) {
        return dynamicFrictionFFVolts * Math.signum(motorRev_S) / saturationVoltage;
    }

    /**
     * Velocity feedforward in duty cycle units [-1, 1]
     */
    private static double velocityFF(double motorRev_S) {
        return velocityFFVoltS_Rev * motorRev_S / saturationVoltage;
    }

    /**
     * Acceleration feedforward in duty cycle units [-1, 1]
     */
    private static double accelFF(double accelM_S_S) {
        return accelFFVoltS2_M * accelM_S_S / saturationVoltage;
    }
}