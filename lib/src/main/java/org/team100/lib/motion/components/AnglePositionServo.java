package org.team100.lib.motion.components;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;

/** Positional control using Rotation2d. */
public class AnglePositionServo implements PositionServo<Rotation2d> {
    public static class Config {
        public double kDeadband = 0.03;
    }

    private final Config m_config = new Config();
    private final Telemetry t = Telemetry.get();
    private final VelocityServo<Angle> m_servo;
    private final Encoder100<Angle> m_encoder;
    private final TrapezoidProfile.Constraints m_constraints;
    private final PIDController m_controller;
    private final double m_period;
    private final String m_name;
    // the profile class is both a stateful follower and
    // a stateless calculator. we use the stateless one so we can make
    // the profile object once.
    private final TrapezoidProfile m_profile;

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    // TODO: use a profile that exposes acceleration and use it.
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    public AnglePositionServo(
            String name,
            VelocityServo<Angle> servo,
            Encoder100<Angle> encoder,
            TrapezoidProfile.Constraints constraints,
            PIDController controller) {
        m_servo = servo;
        m_encoder = encoder;
        m_constraints = constraints;
        m_controller = controller;
        m_period = controller.getPeriod();
        m_name = String.format("/angle position servo %s", name);
        m_profile = new TrapezoidProfile(m_constraints);
    }

    @Override
    public void setPosition(Rotation2d angle) {
        double measurement = getTurningAngleRad();
        m_goal = new TrapezoidProfile.State(angle.getRadians(), 0.0);
        m_setpoint = m_profile.calculate(m_period, m_goal, m_setpoint);
        
        double u_FB = m_controller.calculate(measurement, m_setpoint.position);
        double u_FF = m_setpoint.velocity;
        double u_TOTAL = u_FB + u_FF;
        // TODO: should there be a deadband here?
        u_TOTAL = MathUtil.applyDeadband(u_TOTAL, m_config.kDeadband, m_constraints.maxVelocity);
        u_TOTAL = MathUtil.clamp(u_TOTAL, -m_constraints.maxVelocity, m_constraints.maxVelocity);
        m_servo.setVelocity(u_TOTAL);
        
        t.log(Level.DEBUG, m_name + "/u_FB ", u_FB);
        t.log(Level.DEBUG, m_name + "/u_FF", u_FF);
        t.log(Level.DEBUG, m_name + "/Measurement (rad)", getTurningAngleRad());
        t.log(Level.DEBUG, m_name + "/Measurement (deg)", Units.radiansToDegrees(getTurningAngleRad()));
        t.log(Level.DEBUG, m_name + "/Goal (rad)", m_goal.position);
        t.log(Level.DEBUG, m_name + "/Setpoint (rad)", m_setpoint.position);
        t.log(Level.DEBUG, m_name + "/Setpoint Velocity (rad/s)", m_setpoint.velocity);
        t.log(Level.DEBUG, m_name + "/Error (rad)", m_controller.getPositionError());
        t.log(Level.DEBUG, m_name + "/Error Velocity (rad/s)", m_controller.getVelocityError());
        t.log(Level.DEBUG, m_name + "/Actual Speed", m_servo.getVelocity());
    }

    @Override
    public Rotation2d getPosition() {
        return new Rotation2d(getTurningAngleRad());
    }

    public void stop() {
        m_servo.stop();
    }

    public void close() {
        m_encoder.close();
    }

    public State getSetpoint() {
        return m_setpoint;
    }

    /////////////////////////////////////////////

    private double getTurningAngleRad() {
        return MathUtil.angleModulus(m_encoder.getPosition());
    }
}
