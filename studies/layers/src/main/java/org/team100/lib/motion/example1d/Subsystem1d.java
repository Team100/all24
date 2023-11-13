package org.team100.lib.motion.example1d;

import java.util.function.DoubleConsumer;
import java.util.function.DoublePredicate;
import java.util.function.DoubleUnaryOperator;
import java.util.function.UnaryOperator;

import org.team100.lib.motion.example1d.framework.Actuation;
import org.team100.lib.motion.example1d.framework.Actuator;
import org.team100.lib.motion.example1d.framework.Configuration;
import org.team100.lib.motion.example1d.framework.ConfigurationController;
import org.team100.lib.motion.example1d.framework.Kinematics;
import org.team100.lib.motion.example1d.framework.Workstate;

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
    private final Actuator<Double> m_jointServo;

    /**
     * Source of velocity references. parameters are time (sec) and state (position
     * in meters).
     * 
     * Acts in work space, e.g. cartesian.  Should it?
     * TODO: immutable
     */
    private ProfileFollower m_follower;

    /**
     * Adjusts setpoints for policy, e.g. feasibility. This is useful for manual
     * control, which isn't guaranteed to be feasible.
     * TODO: immutable, generic
     */
    private UnaryOperator<Workstate<Double>> m_filter;

    /** Enables the servo. */
    private DoublePredicate m_enabler;

    private Kinematics<Workstate<Double>, Configuration<Double>> m_kinematics;

    private ConfigurationController<Double, Double> m_confController;

    // TODO: make this generic
    public Subsystem1d(Actuator<Double> servo) {
        m_jointServo = servo;
        m_follower = new ZeroVelocitySupplier1d();
        m_filter = x -> x;
        m_enabler = x -> true;
        // TODO: inject kinematics?
        m_kinematics = new CrankKinematics(1,2);
        // TODO: make this a real class
        m_confController = new ConfigurationController<>(){

            @Override
            public Actuation<Double> calculate(Configuration<Double> config) {
                // for now this is a passthrough which is completely wrong
                // TODO fix it
                return new CrankActuation(config.getConfiguration());
            }
            
        };
    }

    public void setProfileFollower(ProfileFollower follower) {
        if (follower == null)
            throw new IllegalArgumentException("null follower");
        m_follower = follower;
    }

    public ProfileFollower getProfileFollower() {
        return m_follower;
    }

    /** set the workspace filter 
     * TODO: make this generic
    */
    public void setFilter(UnaryOperator<Workstate<Double>> filter) {
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
            m_jointServo.set(new CrankActuation(0));
            return;
        }
        Workstate<Double> workspaceControlM_S = m_follower.apply(new CrankWorkstate(getPositionM()));

        if (m_filter != null) {
            workspaceControlM_S = m_filter.apply(workspaceControlM_S);
        }

        Configuration<Double> conf = m_kinematics.inverse(workspaceControlM_S);

        Actuation<Double> actuation = m_confController.calculate(conf);
        // TODO: add configuration controller here.
        m_jointServo.set(actuation);
    }

}
