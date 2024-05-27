package org.team100.lib.motor.drive;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.Rev100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

/**
 * Linear drive motor using REV Neo.
 * 
 * This is not finished, don't use it without finishing it.
 */
public class NeoVortexDriveMotor implements Motor100<Distance100> {
    /**
     * Motor resistance https://www.revrobotics.com/rev-21-1652/
     */
    private static final double kROhms = 0.057;
    /**
     * Motor torque constant https://www.revrobotics.com/rev-21-1652/
     */
    private static final double kTNm_amp = 0.017;

    private final RelativeEncoder m_encoder;

    private final Feedforward100 m_ff;

    private final Telemetry t = Telemetry.get();
    private final SparkPIDController m_pidController;
    private final CANSparkFlex m_motor;
    private final double m_gearRatio;
    private final double m_wheelDiameter;
    private final String m_name;

    public NeoVortexDriveMotor(
            String name,
            int canId,
            boolean motorPhase,
            int currentLimit,
            double gearRatio,
            double wheelDiameter,
            Feedforward100 ff,
            PIDConstants pid) {
        m_gearRatio = gearRatio;
        m_wheelDiameter = wheelDiameter;
        m_ff = ff;
        m_motor = new CANSparkFlex(canId, MotorType.kBrushless);
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor);
        Rev100.currentConfig(m_motor, currentLimit);

        m_motor.setInverted(!motorPhase);
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
        t.log(Level.TRACE, m_name, "Output", output);
        log();
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    /**
     * Using the supplied wheel diameter and gear ratio, set the motor velocity
     * to the correct RPM given the desired linear speed in m/s.
     * 
     * Supports accel feedforward.
     * 
     * Note the implementation here is surely wrong, it needs to be calibrated.
     */
    @Override
    public void setVelocity(double outputM_S, double accelM_S2) {
        double wheelRev_S = outputM_S / (m_wheelDiameter * Math.PI);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double motorRev_M = motorRev_S * 60;

        double wheelRev_S2 = accelM_S2 / (m_wheelDiameter * Math.PI);
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;

        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double frictionFFVolts = m_ff.frictionFFVolts(m_encoder.getVelocity() / 60, motorRev_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);
        double kFF = frictionFFVolts + velocityFFVolts + accelFFVolts;

        Rev100.warn(() -> m_pidController.setReference(motorRev_M, ControlType.kVelocity, 0, kFF, ArbFFUnits.kVoltage));

        t.log(Level.TRACE, m_name, "friction feedforward volts", frictionFFVolts);
        t.log(Level.TRACE, m_name, "velocity feedforward volts", velocityFFVolts);
        t.log(Level.TRACE, m_name, "accel feedforward volts", accelFFVolts);
        t.log(Level.TRACE, m_name, "desired speed (rev_s)", motorRev_S);
        log();
    }

    @Override
    public void setVelocity(double outputM_S, double accelM_S2, double torqueNm) {
        double wheelRev_S = outputM_S / (m_wheelDiameter * Math.PI);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double motorRev_M = motorRev_S * 60;

        double wheelRev_S2 = accelM_S2 / (m_wheelDiameter * Math.PI);
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;

        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double frictionFFVolts = m_ff.frictionFFVolts(m_encoder.getVelocity() / 60, motorRev_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);

        double torqueFFAmps = torqueNm / kTNm_amp;
        double torqueFFVolts = torqueFFAmps * kROhms;

        double kFF = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        Rev100.warn(() -> m_pidController.setReference(motorRev_M, ControlType.kVelocity, 0, kFF, ArbFFUnits.kVoltage));

        t.log(Level.TRACE, m_name, "friction feedforward volts", frictionFFVolts);
        t.log(Level.TRACE, m_name, "velocity feedforward volts", velocityFFVolts);
        t.log(Level.TRACE, m_name, "accel feedforward volts", accelFFVolts);
        t.log(Level.TRACE, m_name, "torque feedforward volts", torqueFFVolts);
        t.log(Level.TRACE, m_name, "desired speed (rev_s)", motorRev_S);
        log();
    }

    @Override
    public double getTorque() {
        return m_motor.getOutputCurrent() * kTNm_amp;
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * @return integrated sensor position in rotations.
     */
    public double getPositionRot() {
        return m_encoder.getPosition();
    }

    /**
     * @return integrated sensor velocity in RPM
     */
    public double getRateRPM() {
        return m_encoder.getVelocity();
    }

    /**
     * Sets integrated sensor position to zero.
     */
    public void resetPosition() {
        Rev100.warn(() -> m_encoder.setPosition(0));
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
