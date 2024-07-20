package org.team100.frc2024.motion;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.motion.components.AngularPositionServo;
import org.team100.lib.profile.Profile100;

/**
 * Wraps an angular position servo, supplying it with the correct feed forward
 * torque for gravity compensation
 */
public class GravityServo2 implements GravityServoInterface {
    private final AngularPositionServo m_servo;

    public GravityServo2(
            AngularPositionServo servo) {
        m_servo = servo;
    }

    @Override
    public void reset() {
        m_servo.reset();
    }

    @Override
    public OptionalDouble getPositionRad() {
        return m_servo.getPosition();
    }

    @Override
    public void setPosition(double goalRad) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    public void setState(State100 goal) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setState'");
    }

    @Override
    public void stop() {
        m_servo.stop();
    }

    @Override
    public void setProfile(Profile100 profile) {
        m_servo.setProfile(profile);
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTorqueLimit'");
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }

}
