package org.team100.lib.motion.example1d;

import java.util.function.DoubleConsumer;
import java.util.function.DoublePredicate;
import java.util.function.DoubleUnaryOperator;

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
    private final DoubleConsumer m_jointServo;

    /**
     * Source of velocity references. parameters are time (sec) and state (position
     * in meters).
     * 
     * Acts in work space, e.g. cartesian.  Should it?
     */
    private ProfileFollower m_follower;

    /**
     * Adjusts setpoints for policy, e.g. feasibility. This is useful for manual
     * control, which isn't guaranteed to be feasible.
     */
    private DoubleUnaryOperator m_filter;

    /** Enables the servo. */
    private DoublePredicate m_enabler;

    public Subsystem1d(DoubleConsumer servo) {
        m_jointServo = servo;
        m_follower = new ZeroVelocitySupplier1d();
        m_filter = x -> x;
        m_enabler = x -> true;
    }

    public void setProfileFollower(ProfileFollower follower) {
        if (follower == null)
            throw new IllegalArgumentException("null follower");
        m_follower = follower;
    }

    public ProfileFollower getProfileFollower() {
        return m_follower;
    }

    public void setFilter(DoubleUnaryOperator filter) {
        if (filter == null)
            throw new IllegalArgumentException("null filter");
        m_filter = filter;
    }

    public void setEnable(DoublePredicate enabler) {
        if (enabler == null)
            throw new IllegalArgumentException("null enabler");
        m_enabler = enabler;
    }

    public double getPositionM() {
        return 0.0; // in reality this would actually return position
    }

    public double getVelocityM_S() {
        return 0.0; // in reality this would actually return velocity
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
            m_jointServo.accept(0);
            return;
        }
        double workspaceControlM_S = m_follower.apply(getPositionM());

        if (m_filter != null) {
            workspaceControlM_S = m_filter.applyAsDouble(workspaceControlM_S);
        }
        m_jointServo.accept(workspaceControlM_S);
    }

}
