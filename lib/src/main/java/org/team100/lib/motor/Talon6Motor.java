package org.team100.lib.motor;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.IntSupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.CotemporalCache;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Superclass for TalonFX motors.
 */
public abstract class Talon6Motor implements BareMotor {
    private final TalonFX m_motor;
    private final Feedforward100 m_ff;

    // Two levels of caching here: the cotemporal cache caches the value
    // and also the supplier
    protected final Supplier<Double> m_position;
    protected final Supplier<Double> m_velocity;
    protected final Supplier<Double> m_dutyCycle;
    protected final Supplier<Double> m_error;
    protected final Supplier<Double> m_supply;
    protected final Supplier<Double> m_stator;
    protected final Supplier<Double> m_temp;
    protected final Supplier<Double> m_torque;

    // caching the control requests saves allocation
    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
    private final PositionVoltage m_PositionVoltage = new PositionVoltage(0);

    private final double m_supplyLimit;

    // LOGGERS
    private final DoubleSupplierLogger2 m_log_desired_duty;
    private final DoubleSupplierLogger2 m_log_desired_position;
    private final DoubleSupplierLogger2 m_log_desired_speed;
    private final DoubleSupplierLogger2 m_log_desired_accel;
    private final DoubleSupplierLogger2 m_log_friction_FF;
    private final DoubleSupplierLogger2 m_log_velocity_FF;
    private final DoubleSupplierLogger2 m_log_accel_FF;
    private final DoubleSupplierLogger2 m_log_torque_FF;
    private final IntSupplierLogger2 m_log_device_id;
    private final DoubleSupplierLogger2 m_log_velocity;
    private final DoubleSupplierLogger2 m_log_output;
    private final DoubleSupplierLogger2 m_log_error;
    private final DoubleSupplierLogger2 m_log_supply;
    private final DoubleSupplierLogger2 m_log_stator;
    private final DoubleSupplierLogger2 m_log_torque;
    private final DoubleSupplierLogger2 m_log_temp;

    protected Talon6Motor(
            SupplierLogger2 parent,
            int canId,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            PIDConstants lowLevelVelocityConstants,
            Feedforward100 ff) {
        SupplierLogger2 child = parent.child(this);
        m_motor = new TalonFX(canId);
        m_ff = ff;
        m_supplyLimit = supplyLimit;

        TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();
        Phoenix100.baseConfig(talonFXConfigurator);
        Phoenix100.motorConfig(talonFXConfigurator, motorPhase);
        Phoenix100.currentConfig(talonFXConfigurator, supplyLimit, statorLimit);
        Phoenix100.pidConfig(talonFXConfigurator, lowLevelVelocityConstants);

        Phoenix100.crash(() -> m_motor.getPosition().setUpdateFrequency(50));
        Phoenix100.crash(() -> m_motor.getVelocity().setUpdateFrequency(50));
        Phoenix100.crash(() -> m_motor.getTorqueCurrent().setUpdateFrequency(50));

        m_position = new CotemporalCache<>(m_motor.getPosition().refresh()::getValueAsDouble);
        m_velocity = new CotemporalCache<>(m_motor.getVelocity().refresh()::getValueAsDouble);
        m_dutyCycle = new CotemporalCache<>(m_motor.getDutyCycle().refresh()::getValueAsDouble);
        m_error = new CotemporalCache<>(m_motor.getClosedLoopError().refresh()::getValueAsDouble);
        m_supply = new CotemporalCache<>(m_motor.getSupplyCurrent().refresh()::getValueAsDouble);
        m_stator = new CotemporalCache<>(m_motor.getStatorCurrent().refresh()::getValueAsDouble);
        m_temp = new CotemporalCache<>(m_motor.getDeviceTemp().refresh()::getValueAsDouble);
        m_torque = new CotemporalCache<>(m_motor.getTorqueCurrent().refresh()::getValueAsDouble);
       
        m_log_desired_duty = child.doubleLogger(Level.TRACE, "desired duty cycle [-1,1]");
        m_log_desired_position = child.doubleLogger(Level.TRACE, "desired position (rev)");
        m_log_desired_speed = child.doubleLogger(Level.TRACE, "desired speed (rev_s)");
        m_log_desired_accel = child.doubleLogger(Level.TRACE, "desired accel (rev_s2)");
        m_log_friction_FF = child.doubleLogger(Level.TRACE, "friction feedforward (v)");
        m_log_velocity_FF = child.doubleLogger(Level.TRACE, "velocity feedforward (v)");
        m_log_accel_FF = child.doubleLogger(Level.TRACE, "accel feedforward (v)");
        m_log_torque_FF = child.doubleLogger(Level.TRACE, "torque feedforward (v)");
        m_log_device_id = child.intLogger(Level.TRACE, "Device ID");
        m_log_velocity = child.doubleLogger(Level.TRACE, "velocity (rev_s)");
        m_log_output = child.doubleLogger(Level.TRACE, "output [-1,1]");
        m_log_error = child.doubleLogger(Level.TRACE, "error (rev_s)");
        m_log_supply = child.doubleLogger(Level.TRACE, "supply current (A)");
        m_log_stator = child.doubleLogger(Level.TRACE, "stator current (A)");
        m_log_torque = child.doubleLogger(Level.TRACE, "torque (Nm)");
        m_log_temp = child.doubleLogger(Level.TRACE, "temperature (C)");
    }

    @Override
    public void setDutyCycle(double output) {
        Phoenix100.warn(() -> m_motor.setControl(m_dutyCycleOut
                .withOutput(output)));
        m_log_desired_duty.log(() -> output);
        log();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        int currentA = (int) (torqueNm / kTNm_amp());
        TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();
        Phoenix100.currentConfig(talonFXConfigurator, m_supplyLimit, currentA);
    }

    /**
     * Use VelocityVoltage outboard PID control to hold the given velocity, with
     * acceleration and torque feedforwards.
     */
    @Override
    public void setVelocity(double motorRad_S, double motorAccelRad_S2, double motorTorqueNm) {
        double motorRev_S = motorRad_S / (2 * Math.PI);
        double motorRev_S2 = motorAccelRad_S2 / (2 * Math.PI);
        double currentMotorRev_S = m_velocity.get();

        double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);
        double torqueFFVolts = getTorqueFFVolts(motorTorqueNm);

        double kFFVolts = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        // VelocityVoltage has an acceleration field for kA feedforward but we use
        // arbitrary feedforward for that.
        Phoenix100.warn(() -> m_motor.setControl(
                m_velocityVoltage
                        .withVelocity(motorRev_S)
                        .withFeedForward(kFFVolts)));

        m_log_desired_speed.log(() -> motorRev_S);
        m_log_desired_accel.log(() -> motorRev_S2);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> velocityFFVolts);
        m_log_accel_FF.log(() -> accelFFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        log();
    }

    /**
     * Use PositionVoltage outboard PID control to hold the given position, with
     * velocity and torque feedforwards.
     * 
     * Motor revolutions wind up, so setting 0 revs and 1 rev are different.
     */
    @Override
    public void setPosition(double motorPositionRad, double motorVelocityRad_S, double motorTorqueNm) {
        double motorRev = motorPositionRad / (2 * Math.PI);
        double motorRev_S = motorVelocityRad_S / (2 * Math.PI);
        double currentMotorRev_S = m_velocity.get();

        double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double torqueFFVolts = getTorqueFFVolts(motorTorqueNm);

        double kFFVolts = frictionFFVolts + velocityFFVolts + torqueFFVolts;

        // PositionVoltage has a velocity field for kV feedforward but we use arbitrary
        // feedforward for that.
        Phoenix100.warn(() -> m_motor.setControl(
                m_PositionVoltage
                        .withPosition(motorRev)
                        .withFeedForward(kFFVolts)));

        m_log_desired_position.log(() -> motorRev);
        m_log_desired_speed.log(() -> motorRev_S);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> velocityFFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        log();
    }

    @Override
    public double getVelocityRad_S() {
        return getVelocityRev_S() * 2 * Math.PI;
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * Sets integrated sensor position to zero.
     */
    public void resetEncoderPosition() {
        Phoenix100.warn(() -> m_motor.setPosition(0));
    }

    /**
     * Set integrated sensor position in rotations.
     */
    public void setEncoderPosition(double motorPositionRev) {
        Phoenix100.warn(() -> m_motor.setPosition(motorPositionRev));
    }

    @Override
    public void setEncoderPositionRad(double positionRad) {
        double motorPositionRev = positionRad / (2 * Math.PI);
        setEncoderPosition(motorPositionRev);
    }

    public double getVelocityRev_S() {
        return m_velocity.get();
    }

    public double getPositionRev() {
        return m_position.get();
    }

    protected void log() {
        // suppliers here are never touched in the non-logging case.
        m_log_device_id.log(m_motor::getDeviceID);
        m_log_velocity.log(m_velocity);
        m_log_output.log(m_dutyCycle);
        m_log_error.log(m_error);
        m_log_supply.log(m_supply);
        m_log_stator.log(m_stator);
        m_log_torque.log(this::getMotorTorque);
        m_log_temp.log(m_temp);
    }

    private double getMotorTorque() {
        // I looked into latency compensation of this signal but it doesn't seem
        // possible. latency compensation requires a signal and its time derivative,
        // e.g. position and velocity, or yaw and angular velocity. There doesn't seem
        // to be such a thing for current.
        return m_torque.get() * kTNm_amp();
    }

    @Override
    public void periodic() {
        log();
    }
}
