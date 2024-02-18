// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion;

import org.team100.lib.config.SysParam;
import org.team100.lib.controller.State100;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class GravityServo {

    String m_name;
    SysParam m_params;
    PIDController m_controller;
    Profile100 m_profile;
    CANSparkMax m_motor;
    RelativeEncoder m_encoder;
    private final double m_period;
    double m_maxRadsM_S;
    Telemetry t = Telemetry.get();
    double m_gravityScale;



    private State100 m_goal = new State100(0, 0);
    private State100 m_setpoint = new State100(0, 0);

    public GravityServo(
        String name,
        SysParam params,
        PIDController controller,
        Profile100 profile,
        int canID,
        double period,
        double gravityScale
    ){

    m_period = period;
    m_motor = new CANSparkMax(canID, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_name = name;
    m_params = params;
    m_controller = controller;
    m_profile = profile;
    m_gravityScale = gravityScale;

    

    

    }

    public void reset(){
        m_controller.reset();
        m_setpoint = new State100(getPosition(), 0);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public double getRawPosition() {
        return m_encoder.getPosition();
    }
    
    public void setPosition(double goal) {
        double measurement = m_encoder.getPosition();

        // use the modulus closest to the measurement.
        // note zero velocity in the goal.
        m_goal = new State100((goal - measurement) + measurement, 0.0);

        m_setpoint = new State100(
                (m_setpoint.x() - measurement) + measurement,
                m_setpoint.v());

        m_setpoint = m_profile.calculate(m_period, m_setpoint, m_goal);

        double u_FB = m_controller.calculate(measurement, m_setpoint.x());
        double u_FF = m_setpoint.v();

        // note u_FF is rad/s, so a big number, u_FB should also be a big number.
        double gravityTorque = m_gravityScale * Math.cos(MathUtil.angleModulus(m_encoder.getPosition()));
        double u_TOTAL = u_FB + u_FF + gravityTorque;
        // u_TOTAL = MathUtil.clamp(u_TOTAL, -m_maxRadsM_S, m_maxRadsM_S);
        m_motor.set(u_TOTAL * 0.01  ); //rot/s to rpm conversion

        m_controller.setIntegratorRange(0, 0.1);

        t.log(Level.DEBUG, m_name, "u_FB", u_FB);
        t.log(Level.DEBUG, m_name, "u_FF", u_FF);
        t.log(Level.DEBUG, m_name, "u_TOTAL", u_TOTAL);
        t.log(Level.DEBUG, m_name, "Measurement", measurement);
        t.log(Level.DEBUG, m_name, "Goal", m_goal);
        t.log(Level.DEBUG, m_name, "Setpoint", m_setpoint);
        t.log(Level.DEBUG, m_name, "Setpoint Velocity", m_setpoint.v());
        t.log(Level.DEBUG, m_name, "Controller Position Error", m_controller.getPositionError());
        t.log(Level.DEBUG, m_name, "Controller Velocity Error", m_controller.getVelocityError());

    }

    public void periodic(){
        t.log(Level.DEBUG, m_name, "Get Raw Position", getRawPosition());

    }

    public void stop(){
        m_motor.set(0);
    }

   



}
