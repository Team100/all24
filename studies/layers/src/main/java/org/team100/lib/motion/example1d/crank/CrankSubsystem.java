package org.team100.lib.motion.example1d.crank;

import java.util.function.DoublePredicate;
import java.util.function.UnaryOperator;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** This is an example subsystem using the 1d components. */
public class CrankSubsystem extends Subsystem {
    /**
     * Closed loop controller on velocity.
     * 
     * Acts in configuration space, e.g. joints.
     * 
     * TODO: change this type to reflect the "configuration space" type
     */
    private final CrankVelocityServo m_jointServo;

    /**
     * Source of velocity references. parameters are time (sec) and state (position
     * in meters).
     * 
     * Acts in work space, e.g. cartesian. Should it?
     * TODO: immutable
     */
    private CrankProfileFollower m_follower;

    // private WorkspaceController<CrankWorkstate> m_workspaceController;

    /**
     * Adjusts setpoints for policy, e.g. feasibility. This is useful for manual
     * control, which isn't guaranteed to be feasible.
     * TODO: immutable, generic
     */
    private UnaryOperator<CrankWorkstate> m_workspaceFilter;
    private UnaryOperator<CrankActuation> m_actuationFilter;

    /** Enables the servo. */
    private DoublePredicate m_enabler;

    private CrankKinematics m_kinematics;

    private CrankConfigurationController m_confController;

    public CrankSubsystem(
        CrankProfileFollower follower,
        CrankVelocityServo servo) {
        m_jointServo = servo;
        m_follower = follower;
        // m_workspaceController = new IdentityWorkspaceController<>(CrankWorkstate::new);
        m_workspaceFilter = x -> x;
        m_actuationFilter = x -> x;
        m_enabler = x -> true;
        // TODO: inject kinematics?
        m_kinematics = new CrankKinematics(1, 2);

        // m_confController = new CrankConfigurationController();
        m_confController = new CrankConfigurationZero();
    }

    public void setProfileFollower(CrankProfileFollower follower) {
        if (follower == null)
            throw new IllegalArgumentException("null follower");
        m_follower = follower;
    }

    public CrankProfileFollower getProfileFollower() {
        return m_follower;
    }

    /**
     * set the workspace filter
     * TODO: make this generic
     */
    public void setFilter(UnaryOperator<CrankWorkstate> filter) {
        m_workspaceFilter = filter;
    }

    public void setActuationFilter(UnaryOperator<CrankActuation> filter) {
        m_actuationFilter = filter;
    }

    public void setEnable(DoublePredicate enabler) {
        if (enabler == null)
            throw new IllegalArgumentException("null enabler");
        m_enabler = enabler;
    }

    public void setConfigurationController(CrankConfigurationController confController) {
        m_confController = confController;
    }

    private boolean enabled() {
        if (m_follower == null)
            return false;
        if (m_enabler == null)
            return false;
        double measurement = 0.0; // TODO: use a real measurement here.
        return m_enabler.test(measurement);
    }

    @Override
    public void periodic() {
        if (!enabled()) {
            m_jointServo.set(new CrankActuation(0));
            return;
        }

        CrankWorkstate workspaceControlM_S = m_follower.calculate();

        // workspaceControlM_S = m_workspaceController.calculate(measurement, null);

        if (m_workspaceFilter != null) {
            workspaceControlM_S = m_workspaceFilter.apply(workspaceControlM_S);
        }

        CrankConfiguration setpoint = m_kinematics.inverse(workspaceControlM_S);

        // TODO: use a real measurement here
        CrankActuation actuation = m_confController.calculate(
                new CrankConfiguration(0.0),
                setpoint);

        if (m_actuationFilter != null) {
            actuation = m_actuationFilter.apply(actuation);
        }

        m_jointServo.set(actuation);
    }

}
