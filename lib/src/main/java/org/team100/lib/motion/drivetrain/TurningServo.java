package org.team100.lib.motion.drivetrain;

import org.team100.lib.encoder.turning.TurningEncoder;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.turning.TurningMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/** Feedforward and feedback control of a single turning motor. */
public class TurningServo {
    public static class Config {
        public double kSteeringDeadband = 0.03;
    }

    private final Config m_config = new Config();
    private final Telemetry t = Telemetry.get();

    private final Experiments m_experiments;
    private final TurningMotor m_turningMotor;
    private final TurningEncoder m_turningEncoder;
    private final ProfiledPIDController m_turningController;
    private final SimpleMotorFeedforward m_turningFeedforward;
    private final String m_name;

    public TurningServo(
            Experiments experiments,
            String name,
            TurningMotor turningMotor,
            TurningEncoder turningEncoder,
            ProfiledPIDController turningController,
            SimpleMotorFeedforward turningFeedforward) {
        m_experiments = experiments;
        m_turningMotor = turningMotor;
        m_turningEncoder = turningEncoder;
        m_turningController = turningController;
        m_turningFeedforward = turningFeedforward;
        m_name = String.format("/Swerve TurningServo %s", name);
    }

    void setTurning(SwerveModuleState state) {
        if (m_experiments.enabled(Experiment.UseClosedLoopSteering)) {
            offboard(state);
        } else {
            onboard(state);
        }
        log();
    }

    void offboard(SwerveModuleState state) {
        double turningMotorControllerOutputRad_S = m_turningController.calculate(
                getTurningAngleRad(), state.angle.getRadians());
        double turningFeedForwardRad_S = getTurnSetpointVelocityRadS();
        double turnOutputRad_S = turningMotorControllerOutputRad_S + turningFeedForwardRad_S;
        double turnOutputDeadbandRad_S = MathUtil.applyDeadband(turnOutputRad_S, m_config.kSteeringDeadband);
        m_turningMotor.setVelocity(turnOutputDeadbandRad_S, 0);

        t.log(Level.DEBUG, m_name + "/Controller Output rad_s", turningMotorControllerOutputRad_S);
        t.log(Level.DEBUG, m_name + "/Feed Forward Output rad_s", turningFeedForwardRad_S);
        t.log(Level.DEBUG, m_name + "/DESIRED POSITION", state.angle.getRadians());
        t.log(Level.DEBUG, m_name + "/ACTUAL POSITION", getTurningAngleRad());
    }

    void onboard(SwerveModuleState state) {
        double turningMotorControllerOutput = m_turningController.calculate(
                getTurningAngleRad(), state.angle.getRadians());
        double turningFeedForwardOutput = m_turningFeedforward.calculate(getTurnSetpointVelocityRadS(), 0);
        double turnOutput = turningMotorControllerOutput + turningFeedForwardOutput;
        set(MathUtil.applyDeadband(turnOutput, m_config.kSteeringDeadband));

        t.log(Level.DEBUG, m_name + "/Controller Output", turningMotorControllerOutput);
        t.log(Level.DEBUG, m_name + "/Feed Forward Output", turningFeedForwardOutput);
        t.log(Level.DEBUG, m_name + "/Total Output", turningFeedForwardOutput);
        t.log(Level.DEBUG, m_name + "/Actual Speed", m_turningMotor.get());
    }

    private void log() {
        t.log(Level.DEBUG, m_name + "/Turning Measurement (rad)", getTurningAngleRad());
        t.log(Level.DEBUG, m_name + "/Turning Measurement (deg)", Units.radiansToDegrees(getTurningAngleRad()));
        t.log(Level.DEBUG, m_name + "/Turning Goal (rad)", m_turningController.getGoal().position);
        t.log(Level.DEBUG, m_name + "/Turning Setpoint (rad)", m_turningController.getSetpoint().position);
        t.log(Level.DEBUG, m_name + "/Turning Setpoint Velocity (rad/s)", getTurnSetpointVelocityRadS());
        t.log(Level.DEBUG, m_name + "/Turning Error (rad)", m_turningController.getPositionError());
        t.log(Level.DEBUG, m_name + "/Turning Error Velocity (rad/s)", m_turningController.getVelocityError());
        t.log(Level.DEBUG, m_name + "/Turning Motor Output [-1, 1]", m_turningMotor.get());
    }

    void set(double output) {
        m_turningMotor.setDutyCycle(output);
    }

    double getTurnSetpointVelocityRadS() {
        return m_turningController.getSetpoint().velocity;
    }

    double getTurningAngleRad() {
        return MathUtil.angleModulus(m_turningEncoder.getAngle());
    }

    Rotation2d getTurningRotation() {
        return new Rotation2d(getTurningAngleRad());
    }

    public void close() {
        m_turningEncoder.close();
    }
}
