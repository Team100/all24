package org.team100.lib.commands.arm;

import java.util.function.DoubleSupplier;

import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive the arm to the specified cartesian position.
 * 
 * There are several flaws in this implementation.
 */
public class CartesianManualPositionalArm extends Command {
    private final Telemetry t = Telemetry.get();

    private final ArmSubsystem m_arm;
    private final ArmKinematics m_kinematics;
    private final DoubleSupplier m_x;
    private final DoubleSupplier m_y;

    private final PIDController m_lowerController;
    private final PIDController m_upperController;

    public CartesianManualPositionalArm(
            ArmSubsystem arm,
            ArmKinematics kinematics,
            DoubleSupplier x,
            DoubleSupplier y) {
        m_arm = arm;
        m_kinematics = kinematics;
        m_x = x;
        m_y = y;
        m_lowerController = new PIDController(1, 0, 0);
        m_upperController = new PIDController(1, 0, 0);
        m_lowerController.enableContinuousInput(-Math.PI, Math.PI);
        m_upperController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(arm);
    }

    /**
     * Use inverse kinematics to transform the manual input into joint space.
     * This uses an offset to keep the inputs away from the origin.
     */
    @Override
    public void execute() {
        Translation2d input = new Translation2d(
                0.6 * m_x.getAsDouble() + 0.7,
                0.6 * m_y.getAsDouble() + 0.7);
        t.log(Level.DEBUG, "/arm position/input", input);

        ArmAngles setpoint = m_kinematics.inverse(input);
        if (setpoint == null) {
            System.out.println("WARNING: ignoring infeasible input");
            return;
        }
        t.log(Level.DEBUG, "/arm position/setpoint", setpoint);

        ArmAngles measurement = m_arm.getPosition();
        t.log(Level.DEBUG, "/arm position/measurement", measurement);

        Translation2d cartesian_measurement = m_kinematics.forward(measurement);
        t.log(Level.DEBUG, "/arm position/cartesian_measurement", cartesian_measurement);

        double u1 = MathUtil.clamp(
                m_lowerController.calculate(measurement.th1, setpoint.th1), -1, 1);
        double u2 = MathUtil.clamp(
                m_upperController.calculate(measurement.th2, setpoint.th2), -1, 1);

        t.log(Level.DEBUG, "/arm position/output/u1", u1);
        t.log(Level.DEBUG, "/arm position/output/u2", u2);

        t.log(Level.DEBUG, "/arm position/error/e1", m_lowerController.getPositionError());
        t.log(Level.DEBUG, "/arm position/error/e2", m_upperController.getPositionError());

        m_arm.set(u1, u2);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.set(0, 0);
    }

}
