package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LaundryArm extends Subsystem {
    // TODO: try lower gear ratios
    private static final int kGearRatio = 15;
    private static final double kOutput = 1.0;

    private final BooleanSupplier m_dumpControl;
    private final ProfiledPIDController m_controller;
    private final CANSparkMax m_motor;

    private final DoublePublisher goalPub;
    private final DoublePublisher measurementPub;
    private final DoublePublisher outputPub;

    private double m_goalTurns;
    private boolean m_enabled;
    private boolean m_placeFinished;

    /**
     * @param dumpControl supplies true to dump the basket
     * @param controller  holds at the "dump" or "level" setting
     * @param motor       we use the built-in encoder
     */
    public LaundryArm(
            BooleanSupplier dumpControl,
            ProfiledPIDController controller,
            CANSparkMax motor) {
        m_dumpControl = dumpControl;
        m_controller = controller;
        m_motor = motor;
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setSmartCurrentLimit(10);
        m_motor.setIdleMode(IdleMode.kCoast);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("arm");
        goalPub = table.getDoubleTopic("goalTurns").publish();
        measurementPub = table.getDoubleTopic("measurementTurns").publish();
        outputPub = table.getDoubleTopic("output1_1").publish();

        m_goalTurns = 0;
        m_enabled = false;
    }

    public void enable() {
        m_motor.getEncoder().setPosition(0);
        m_enabled = true;
    }

    public void disable() {
        m_enabled = false;
        m_motor.set(0);
        outputPub.set(0);
    }

    public void autonomousInit() {
        m_placeFinished = false;
        dump();
    }

    public void autonomousPeriodic() {
        double measurementTurns = getMeasurementTurns();
        if (Math.abs(m_goalTurns-measurementTurns) < .1) {
            m_placeFinished = true;
            level();
        }
        double output = MathUtil.clamp(m_controller.calculate(measurementTurns, m_goalTurns), -1.0*kOutput, kOutput);
        m_motor.set(output);
        outputPub.set(output);
    }

    public boolean placeFinished() {
        return m_placeFinished;
    }

    @Override
    public void periodic() {
        if (m_dumpControl.getAsBoolean()) {
            dump();
        } else {
            level();
        }
        double measurementTurns = getMeasurementTurns();
        double output = m_controller.calculate(measurementTurns, m_goalTurns);
        double angleOffsetRadians = 0.75;
        double measurementRadians = Units.rotationsToRadians(measurementTurns) ;
        double angleRadians = -1.0 * measurementRadians + angleOffsetRadians;
        double gravityTorque = -0.06 * Math.cos(angleRadians);
        double uTotal = output + gravityTorque;
        uTotal = MathUtil.clamp(uTotal, -1.0 * kOutput, kOutput);
        if (m_enabled) {
            m_motor.set(uTotal);
            outputPub.set(uTotal);
        }
    }

    //////////////////////////////////////////////////////////////////

    private void level() {
        setGoalTurns(0);
    }

    private void dump() {
        setGoalTurns(-0.25);
    }

    private void setGoalTurns(double goalTurns) {
        m_goalTurns = goalTurns;
        goalPub.set(m_goalTurns);
    }

    private double getMeasurementTurns() {
        double absolute = m_motor.getEncoder().getPosition();
        double turns = absolute / kGearRatio;
        measurementPub.set(turns);
        return turns;
    }

}
