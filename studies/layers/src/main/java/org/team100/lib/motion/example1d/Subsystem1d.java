package org.team100.lib.motion.example1d;

import java.util.function.DoublePredicate;
import java.util.function.UnaryOperator;

import org.team100.lib.motion.example1d.crank.CrankActuation;
import org.team100.lib.motion.example1d.crank.CrankConfiguration;
import org.team100.lib.motion.example1d.crank.CrankConfigurationController;
import org.team100.lib.motion.example1d.crank.CrankKinematics;
import org.team100.lib.motion.example1d.crank.CrankWorkstate;
import org.team100.lib.motion.example1d.framework.Actuator;
import org.team100.lib.motion.example1d.framework.ConfigurationController;
import org.team100.lib.motion.example1d.framework.Kinematics;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** This is an example subsystem using the 1d components. */
public class Subsystem1d extends Subsystem {
    /**
     * Closed loop controller on velocity.
     * 
     * Acts in configuration space, e.g. joints.
     * 
     * TODO: change this type to reflect the "configuration space" type
     */
    private final Actuator<CrankActuation> m_jointServo;

    /**
     * Source of velocity references. parameters are time (sec) and state (position
     * in meters).
     * 
     * Acts in work space, e.g. cartesian. Should it?
     * TODO: immutable
     */
    private ProfileFollower<CrankWorkstate> m_follower;

    /**
     * Adjusts setpoints for policy, e.g. feasibility. This is useful for manual
     * control, which isn't guaranteed to be feasible.
     * TODO: immutable, generic
     */
    private UnaryOperator<CrankWorkstate> m_filter;

    /** Enables the servo. */
    private DoublePredicate m_enabler;

    private Kinematics<CrankWorkstate, CrankConfiguration> m_kinematics;

    private ConfigurationController<CrankConfiguration, CrankActuation> m_confController;

    // TODO: make this generic
    public Subsystem1d(Actuator<CrankActuation> servo) {
        m_jointServo = servo;
        m_follower = new ZeroVelocitySupplier1d<>(CrankWorkstate::new);
        m_filter = x -> x;
        m_enabler = x -> true;
        // TODO: inject kinematics?
        m_kinematics = new CrankKinematics(1, 2);

        m_confController = new CrankConfigurationController();
    }

    public void setProfileFollower(ProfileFollower<CrankWorkstate> follower) {
        if (follower == null)
            throw new IllegalArgumentException("null follower");
        m_follower = follower;
    }

    public ProfileFollower<CrankWorkstate> getProfileFollower() {
        return m_follower;
    }

    /**
     * set the workspace filter
     * TODO: make this generic
     */
    public void setFilter(UnaryOperator<CrankWorkstate> filter) {
        if (filter == null)
            throw new IllegalArgumentException("null filter");
        m_filter = filter;
    }

    public void setEnable(DoublePredicate enabler) {
        if (enabler == null)
            throw new IllegalArgumentException("null enabler");
        m_enabler = enabler;
    }

    /** fake for the example; they should actually measure something. */
    public double getPositionM() {
        return 0.0;
    }

    /** fake for the example; they should actually measure something. */
    public double getVelocityM_S() {
        return 0.0;
    }

    private boolean enabled() {
        if (m_follower == null)
            return false;
        if (m_enabler == null)
            return false;
        return m_enabler.test(getPositionM());
    }

    @Override
    public void periodic() {
        if (!enabled()) {
            m_jointServo.set(new CrankActuation(0));
            return;
        }
        CrankWorkstate workspaceControlM_S = m_follower.apply(new CrankWorkstate(getPositionM()));

        if (m_filter != null) {
            workspaceControlM_S = m_filter.apply(workspaceControlM_S);
        }

        CrankConfiguration setpoint = m_kinematics.inverse(workspaceControlM_S);

        CrankActuation actuation = m_confController.calculate(
                new CrankConfiguration(getPositionM()),
                setpoint);

        m_jointServo.set(actuation);
    }

}
