package org.team100.frc2024.motion.intake;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.Speeding;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

/**
 * Four-axle wheeled intake with reduction
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * Reduction of 3:1 so 33 turn/s wheel speed.
 * diameter of 0.05m => 0.15 m/turn
 * therefore top speed is around 5 m/s.
 * 
 * This system has low intertia but a lot of friction,
 * and it's fragile. guess at a reasonable accel limit.
 * 
 * TODO: add intake to selftest.
 */
public class IntakeWheel extends Intake implements Speeding {
    /**
     * Surface velocity of whatever is turning in the intake.
     */
    final double kIntakeVelocityM_S = 3;
    private final String m_name;
    private final LimitedVelocityServo<Distance100> intakeMotor;
    private final SpeedingVisualization m_viz;

    public IntakeWheel(int wheelID) {
        m_name = Names.name(this);

        SysParam params = SysParam.limitedNeoVelocityServoSystem(
                3.0,
                0.05,
                5.0,
                20.0,
                -20.0);
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                intakeMotor = ServoFactory.limitedNeoVelocityServo(
                        m_name, wheelID, false, params);
                break;
            case BLANK:
            default:
                intakeMotor = ServoFactory.limitedSimulatedVelocityServo(
                        m_name, params);
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    @Override
    public void intake() {
        intakeMotor.setVelocity(kIntakeVelocityM_S);
    }

    @Override
    public void outtake() {
        intakeMotor.setVelocity(-1.0 * kIntakeVelocityM_S);
    }

    @Override
    public void stop() {
        intakeMotor.stop();
    }


    @Override
    public void periodic() {
        intakeMotor.periodic();
        m_viz.periodic();
    }

    @Override
    public double getVelocity() {
        return intakeMotor.getVelocity();
    }
}