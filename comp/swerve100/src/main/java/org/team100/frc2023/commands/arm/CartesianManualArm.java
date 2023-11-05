package org.team100.frc2023.commands.arm;

import java.util.function.Supplier;

import org.team100.frc2023.subsystems.arm.ArmSubsystem;
import org.team100.lib.motion.arm.ArmKinematics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Manual arm control in cartesian coordinates. */
public class CartesianManualArm extends Command {
    private final ArmSubsystem m_arm;
    private final ArmKinematics m_armKinematicsM;
    private final Supplier<Double> m_dxM_S;
    private final Supplier<Double> m_dyM_S;

    /**
     * @param armKinematicsM in meters
     * @param dxM_S          in meters per second. use Control.armX().
     * @param dyM_S          in meters per second. use Control.armY().
     */
    public CartesianManualArm(ArmSubsystem arm, ArmKinematics armKinematicsM, Supplier<Double> dxM_S,
            Supplier<Double> dyM_S) {
        m_arm = arm;
        m_armKinematicsM = armKinematicsM;
        m_dxM_S = dxM_S;
        m_dyM_S = dyM_S;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setControlNormal();
    }

    /** Set a new reference equal to the current state plus the controller input. */
    @Override
    public void execute() {
        final double dt = 0.02;
        Translation2d currentM = m_armKinematicsM.forward(m_arm.getMeasurement());
        double xReferenceM = currentM.getX() + dt * m_dxM_S.get();
        double yReferenceM = currentM.getY() + dt * m_dyM_S.get();
        m_arm.setReference(m_armKinematicsM.inverse(new Translation2d(xReferenceM, yReferenceM)));
    }

    /** Set the reference equal to the current state. */
    @Override
    public void end(boolean interrupted) {
        m_arm.setReference(m_arm.getMeasurement());
    }
}
