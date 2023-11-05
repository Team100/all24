package org.team100.frc2023.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LaundryArm extends Subsystem {
    private final ProfiledPIDController m_controller;
    private final CANSparkMax m_motor;
    private final int m_gearRatio;

    private final DoublePublisher goalPub;
    private final DoublePublisher measurementPub;
    private final DoublePublisher outputPub;

    private double m_goalTurns;
    private boolean m_enabled;

    /**
     * @param controller holds at the "dump" or "level" setting
     * @param motor      we use the built-in encoder
     * @param gearRatio  between the motor and the arm itself
     */
    public LaundryArm(ProfiledPIDController controller, CANSparkMax motor, int gearRatio) {
        m_controller = controller;
        m_motor = motor;
        m_gearRatio = gearRatio;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("arm");
        goalPub = table.getDoubleTopic("goalTurns").publish();
        measurementPub = table.getDoubleTopic("measurementTurns").publish();
        outputPub = table.getDoubleTopic("output1_1").publish();

        m_goalTurns = 0;
        m_enabled = false;
    }

    /** NOTE! This is where the zero position is set! */
    public void enable() {
        m_motor.getEncoder().setPosition(0);
        m_enabled = true;
    }

    public void disable() {
        m_enabled = false;
        m_motor.set(0);
        outputPub.set(0);
    }

    public void dump() {
        setGoalTurns(-0.25);
    }

    public void level() {
        setGoalTurns(0);
    }

    public boolean atGoal() {
        return m_controller.atGoal();
    }

    @Override
    public void periodic() {
        double measurementTurns = getMeasurementTurns();
        double output = MathUtil.clamp(m_controller.calculate(measurementTurns, m_goalTurns), -1, 1);
        if (m_enabled) {
            m_motor.set(output);
            outputPub.set(output);
        }
    }

    //////////////////////////////////////////////////////////////////

    private void setGoalTurns(double goalTurns) {
        m_goalTurns = goalTurns;
        goalPub.set(m_goalTurns);
    }

    private double getMeasurementTurns() {
        double absolute = m_motor.getEncoder().getPosition();
        double turns = absolute / m_gearRatio;
        measurementPub.set(turns);
        return turns;
    }
}
