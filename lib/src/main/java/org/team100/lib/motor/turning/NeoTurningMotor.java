package org.team100.lib.motor.turning;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Rev100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

/**
 * Swerve steering motor using REV Neo.
 * 
 * This is not finished, don't use it without finishing it.
 */
public class NeoTurningMotor implements Motor100<Angle100> {
    /**
     * Motor resistance https://www.revrobotics.com/rev-21-1650/
     */
    private static final double kROhms = 0.114;
    /**
     * Motor torque constant https://www.revrobotics.com/rev-21-1650/
     */
    private static final double kTNm_amp = 0.028;

    private final RelativeEncoder m_encoder;

    /**
     * This is surely wrong.
     */
    private final double m_gearRatio;

    private final Feedforward100 m_ff;

    /**
     * Proportional feedback coefficient for the controller.
     * 
     * This is a guess. Calibrate it before using it.
     */

    private final Telemetry t = Telemetry.get();
    private final SparkPIDController m_pidController;
    private final CANSparkMax m_motor;
    private final String m_name;

    public NeoTurningMotor(String name,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            Feedforward100 ff,
            PIDConstants pid) {
        m_gearRatio = gearRatio;
        m_ff = ff;

        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor);
        Rev100.currentConfig(m_motor, currentLimit);

        m_motor.setInverted(motorPhase == MotorPhase.REVERSE);
        Rev100.crash(() -> m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20));
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        Rev100.pidConfig(m_pidController, pid);

        m_name = Names.append(name, this);

        t.log(Level.TRACE, m_name, "Device ID", m_motor.getDeviceId());
        t.register(Level.TRACE, m_name, "P", pid.getP(), this::setP);
        t.register(Level.TRACE, m_name, "I", pid.getI(), this::setI);
        t.register(Level.TRACE, m_name, "D", pid.getD(), this::setD);
        t.register(Level.TRACE, m_name, "IZone", pid.getIZone(), this::setIZone);
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.log(Level.TRACE, m_name, "desired duty cycle [-1,1]", output);
        log();
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    /**
     * Supports accel feedforward.
     * 
     * Note the implementation here is surely wrong, it needs to be calibrated.
     */
    @Override
    public void setVelocity(double outputRad_S, double accelRad_S2) {
        double motorRad_S = m_gearRatio * outputRad_S;
        double motorRevs_S = motorRad_S / (2 * Math.PI);
        double motorRevs_M = motorRevs_S * 60;
        double motorRad_S2 = m_gearRatio * accelRad_S2;
        double motorRevs_S2 = motorRad_S2 / (2 * Math.PI);
        double velocityFF = m_ff.velocityFFVolts(motorRevs_S);
        double frictionFF = m_ff.frictionFFVolts(m_encoder.getVelocity() / 60, motorRevs_S);
        double accelFF = m_ff.accelFFVolts(motorRevs_S2);
        double kFF = frictionFF + velocityFF + accelFF;

        Rev100.warn(
                () -> m_pidController.setReference(motorRevs_M, ControlType.kVelocity, 0, kFF, ArbFFUnits.kVoltage));

        t.log(Level.TRACE, m_name, "friction feedforward [-1,1]", frictionFF);
        t.log(Level.TRACE, m_name, "velocity feedforward [-1,1]", velocityFF);
        t.log(Level.TRACE, m_name, "accel feedforward [-1,1]", accelFF);
        t.log(Level.TRACE, m_name, "desired speed (rev_s)", motorRevs_S);
        log();
    }

    @Override
    public void setVelocity(double outputRad_S, double accelRad_S2, double torqueNm) {
        double motorRad_S = m_gearRatio * outputRad_S;
        double motorRevs_S = motorRad_S / (2 * Math.PI);
        double motorRevs_M = motorRevs_S * 60;
        double motorRad_S2 = m_gearRatio * accelRad_S2;
        double motorRevs_S2 = motorRad_S2 / (2 * Math.PI);

        double velocityFFVolts = m_ff.velocityFFVolts(motorRevs_S);
        double frictionFFVolts = m_ff.frictionFFVolts(m_encoder.getVelocity() / 60, motorRevs_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRevs_S2);

        double torqueFFAmps = torqueNm / kTNm_amp;
        double torqueFFVolts = torqueFFAmps * kROhms;

        double kFF = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        Rev100.warn(
                () -> m_pidController.setReference(motorRevs_M, ControlType.kVelocity, 0, kFF, ArbFFUnits.kVoltage));

        t.log(Level.TRACE, m_name, "friction feedforward volts", frictionFFVolts);
        t.log(Level.TRACE, m_name, "velocity feedforward volts", velocityFFVolts);
        t.log(Level.TRACE, m_name, "accel feedforward volts", accelFFVolts);
        t.log(Level.TRACE, m_name, "torque feedforward volts", torqueFFVolts);
        t.log(Level.TRACE, m_name, "desired speed (rev_s)", motorRevs_S);
        log();
    }

    @Override
    public double getTorque() {
        return m_motor.getOutputCurrent() * kTNm_amp;
    }

    public void resetPosition() {
        Rev100.warn(() -> m_encoder.setPosition(0));
    }

    @Override
    public void close() {
        m_motor.close();
    }

    public double getPositionRot() {
        return m_encoder.getPosition();
    }

    public double getRateRPM() {
        return m_encoder.getVelocity();
    }

    public void log() {
        t.log(Level.TRACE, m_name, "position (rev)", m_encoder.getPosition());
        t.log(Level.TRACE, m_name, "velocity (rev_s)", m_encoder.getVelocity() / 60);
        t.log(Level.TRACE, m_name, "current (A)", m_motor.getOutputCurrent());
        t.log(Level.TRACE, m_name, "duty cycle", m_motor.getAppliedOutput());
        t.log(Level.TRACE, m_name, "temperature (C)", m_motor.getMotorTemperature());
    }

    /////////////////////////////////////////////////////////////////

    private void setP(double p) {
        Rev100.warn(() -> m_pidController.setP(p));
    }

    private void setI(double i) {
        Rev100.warn(() -> m_pidController.setI(i));
    }

    private void setD(double d) {
        Rev100.warn(() -> m_pidController.setD(d));
    }

    private void setIZone(double iz) {
        Rev100.warn(() -> m_pidController.setIZone(iz));
    }
}