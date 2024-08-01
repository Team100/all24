package org.team100.lib.motion.components;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;

import org.team100.lib.controller.State100;
import org.team100.lib.profile.Profile100;

public class SelectableAngularPositionServo implements AngularPositionServo {
    private final AngularPositionServo m_whenTrue;
    private final AngularPositionServo m_whenFalse;
    private final BooleanSupplier m_selector;

    public SelectableAngularPositionServo(
            AngularPositionServo whenTrue,
            AngularPositionServo whenFalse,
            BooleanSupplier selector) {
        m_whenTrue = whenTrue;
        m_whenFalse = whenFalse;
        m_selector = selector;
    }

    @Override
    public void reset() {
        if (m_selector.getAsBoolean()) {
            m_whenTrue.reset();
        } else {
            m_whenFalse.reset();
        }
    }

    @Override
    public void setProfile(Profile100 profile) {
        m_whenTrue.setProfile(profile);
        m_whenFalse.setProfile(profile);
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_whenTrue.setTorqueLimit(torqueNm);
        m_whenFalse.setTorqueLimit(torqueNm);
    }

    @Override
    public void setPositionWithVelocity(double goal, double goalVelocity, double feedForwardTorqueNm) {
        if (m_selector.getAsBoolean()) {
            m_whenTrue.setPositionWithVelocity(goal, goalVelocity, feedForwardTorqueNm);
        } else {
            m_whenFalse.setPositionWithVelocity(goal, goalVelocity, feedForwardTorqueNm);
        }
    }

    @Override
    public void setPosition(double goal, double feedForwardTorqueNm) {
        if (m_selector.getAsBoolean()) {
            m_whenTrue.setPosition(goal, feedForwardTorqueNm);
        } else {
            m_whenFalse.setPosition(goal, feedForwardTorqueNm);
        }
    }

    @Override
    public OptionalDouble getPosition() {
        if (m_selector.getAsBoolean()) {
            return m_whenTrue.getPosition();
        } else {
            return m_whenFalse.getPosition();
        }
    }

    @Override
    public OptionalDouble getVelocity() {
        if (m_selector.getAsBoolean()) {
            return m_whenTrue.getVelocity();
        } else {
            return m_whenFalse.getVelocity();
        }
    }

    @Override
    public boolean atSetpoint() {
        if (m_selector.getAsBoolean()) {
            return m_whenTrue.atSetpoint();
        } else {
            return m_whenFalse.atSetpoint();
        }
    }

    @Override
    public boolean atGoal() {
        if (m_selector.getAsBoolean()) {
            return m_whenTrue.atGoal();
        } else {
            return m_whenFalse.atGoal();
        }
    }

    @Override
    public double getGoal() {
        if (m_selector.getAsBoolean()) {
            return m_whenTrue.getGoal();
        } else {
            return m_whenFalse.getGoal();
        }
    }

    @Override
    public void stop() {
        if (m_selector.getAsBoolean()) {
            m_whenTrue.stop();
        } else {
            m_whenFalse.stop();
        }
    }

    @Override
    public void close() {
        if (m_selector.getAsBoolean()) {
            m_whenTrue.close();
        } else {
            m_whenFalse.close();
        }
    }

    @Override
    public State100 getSetpoint() {
        if (m_selector.getAsBoolean()) {
            return m_whenTrue.getSetpoint();
        } else {
            return m_whenFalse.getSetpoint();
        }
    }

    @Override
    public void periodic() {
        if (m_selector.getAsBoolean()) {
            m_whenTrue.periodic();
        } else {
            m_whenFalse.periodic();
        }
    }

    @Override
    public void setVelocity(double goalVelocityRad_S, double feedForwardTorqueNm) {
        if (m_selector.getAsBoolean()) {
            m_whenTrue.setVelocity(goalVelocityRad_S, feedForwardTorqueNm);
        } else {
            m_whenFalse.setVelocity(goalVelocityRad_S, feedForwardTorqueNm);
        }
    }

}
