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
 * Direct-drive roller intake
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.05m => 0.15 m/turn
 * therefore top speed is around 15 m/s.
 * 
 * This system has very low intertia, so can spin up
 * very fast, but it's fragile: limit accel to avoid stressing it.
 * 
 * TODO: add intake to selftest.
 */
public class IntakeRoller extends Intake implements Speeding {
    /**
     * Surface velocity of whatever is turning in the intake.
     */
    final double kIntakeVelocityM_S = 3;

    private final String m_name;
    private final LimitedVelocityServo<Distance100> topRoller;
    private final LimitedVelocityServo<Distance100> bottomRoller;
    private final SpeedingVisualization m_viz;

    public IntakeRoller(int topCAN, int bottomCAN) {
        m_name = Names.name(this);
        SysParam rollerParameter = SysParam.limitedNeoVelocityServoSystem(
                1,
                0.05,
                15,
                10,
                -10);

        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                topRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Top Roller",
                        topCAN,
                        false,
                        rollerParameter);
                bottomRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Bottom Roller",
                        bottomCAN,
                        false,
                        rollerParameter);
                break;
            case BLANK:
            default:
                topRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Top Roller",
                        rollerParameter);
                bottomRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Bottom Roller",
                        rollerParameter);
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    @Override
    public void intake() {
        topRoller.setVelocity(kIntakeVelocityM_S);
        bottomRoller.setVelocity(kIntakeVelocityM_S);
    }

    @Override
    public void outtake() {
        topRoller.setVelocity(-1.0 * kIntakeVelocityM_S);
        bottomRoller.setVelocity(-1.0 * kIntakeVelocityM_S);
    }

    @Override
    public void stop() {
        topRoller.stop();
        bottomRoller.stop();
    }

    @Override
    public void periodic() {
        topRoller.periodic();
        bottomRoller.periodic();
        m_viz.periodic();
    }

    @Override
    public double getVelocity() {
        return (topRoller.getVelocity() + bottomRoller.getVelocity()) / 2;
    }
}
