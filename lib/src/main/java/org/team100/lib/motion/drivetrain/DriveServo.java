package org.team100.lib.motion.drivetrain;

import org.team100.lib.encoder.drive.DriveEncoder;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.drive.DriveMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Feedforward and feedback control of a single drive motor. */
public class DriveServo {
    public static class Config {
        public double kDriveDeadband = 0.03;
    }

    private final Config m_config = new Config();
    private final Telemetry t = Telemetry.get();

    private final Experiments m_experiments;
    private final DriveMotor m_driveMotor;
    private final DriveEncoder m_driveEncoder;
    private final PIDController m_driveController;
    private final SimpleMotorFeedforward m_driveFeedforward;
    private final String m_name;

    // for calculating acceleration
    private double previousSpeedM_S = 0;

    public DriveServo(
            Experiments experiments,
            String name,
            DriveMotor driveMotor,
            DriveEncoder driveEncoder,
            PIDController driveController,
            SimpleMotorFeedforward driveFeedforward) {
        m_experiments = experiments;
        m_driveMotor = driveMotor;
        m_driveEncoder = driveEncoder;
        m_driveController = driveController;
        m_driveFeedforward = driveFeedforward;
        m_name = String.format("/Swerve DriveServo %s", name);
    }

    void setVelocity(double speedM_S) {
        if (m_experiments.enabled(Experiment.UseClosedLoopDrive)) {
            offboard(speedM_S);
        } else {
            onboard(speedM_S);
        }
        log();
    }

    /** Set raw output directly. */
    void set(double output) {
        m_driveMotor.setDutyCycle(output);
    }

    void offboard(double speedM_S) {
        m_driveMotor.setVelocity(speedM_S);
    }

    void onboard(double speedM_S) {
        double accelM_S2 = (speedM_S - previousSpeedM_S) / 0.02; // TODO: measured dt
        previousSpeedM_S = speedM_S;
        double driveMotorControllerOutput = m_driveController.calculate(getDriveSpeedMS(), speedM_S);
        double driveFeedForwardOutput = m_driveFeedforward.calculate(speedM_S, accelM_S2);
        double driveOutput = driveMotorControllerOutput + driveFeedForwardOutput;
        // output deadband to prevent shivering.
        m_driveMotor.setDutyCycle(MathUtil.clamp(
                MathUtil.applyDeadband(driveOutput, m_config.kDriveDeadband), -1, 1));

        t.log(Level.DEBUG, m_name + "Controller Output", driveMotorControllerOutput);
        t.log(Level.DEBUG, m_name + "Feed Forward Output", driveFeedForwardOutput);
    }

    double getDriveDistanceM() {
        return m_driveEncoder.getDistance();
    }

    double getDriveSpeedMS() {
        return m_driveEncoder.getRate();
    }

    private void log() {
        t.log(Level.DEBUG, m_name + "Drive position (m)", m_driveEncoder.getDistance());
        t.log(Level.DEBUG, m_name + "Drive Speed (m_s)", getDriveSpeedMS());
        t.log(Level.DEBUG, m_name + "Drive Setpoint (m_s)", m_driveController.getSetpoint());
        t.log(Level.DEBUG, m_name + "Drive Speed Error (m_s)", m_driveController.getPositionError());
        t.log(Level.DEBUG, m_name + "Drive Accel Error (m_s_s)", m_driveController.getVelocityError());
        t.log(Level.DEBUG, m_name + "Drive Motor Output [-1, 1]", m_driveMotor.get());
    }
}
