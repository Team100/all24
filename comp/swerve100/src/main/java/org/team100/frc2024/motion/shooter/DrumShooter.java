package org.team100.frc2024.motion.shooter;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.Speeding;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

/**
 * Direct-drive shooter with top and bottom drums.
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.1m => 0.314 m/turn
 * therefore top speed is around 30 m/s.
 * 
 * Empirically it seems to take a second or so to spin
 * up, so set the acceleration a bit higher than that to start.
 * 
 * TODO: add shooter to self-test
 */
public class DrumShooter extends Shooter implements Speeding {
    /**
     * Muzzle velocity of game piece exiting the shooter.
     * 
     * The shooter should do whatever is necessary to achieve this;
     * a good approximation for a sticky shooter is the surface
     * speed of whatever wheels are contacting the game piece,
     * but there are many factors that affect the relationship in
     * the real world.
     */
    private static final double kMuzzleVelocityM_S = 15;
    private final String m_name;
    private final LimitedVelocityServo<Distance100> topRoller;
    private final LimitedVelocityServo<Distance100> bottomRoller;
    private final SpeedingVisualization m_viz;

    public DrumShooter(int topRollerID, int bottomRollerID) {
        m_name = Names.name(this);
        SysParam params = SysParam.limitedNeoVelocityServoSystem(
                1,
                0.1,
                30,
                40,
                -40);
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                topRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Top",
                        topRollerID,
                        true,
                        params);
                bottomRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Bottom",
                        bottomRollerID,
                        false,
                        params);
                break;
            case BLANK:
            default:
                topRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Top",
                        params);
                bottomRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Bottom",
                        params);
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    @Override
    public void forward() {
        topRoller.setVelocity(kMuzzleVelocityM_S);
        bottomRoller.setVelocity(kMuzzleVelocityM_S);
    }

    @Override
    public void stop() {
        topRoller.stop();
        bottomRoller.stop();
    }

    public double getFirstRollerVelocity() {
        return topRoller.getVelocity();
    }
    
    public double getSecondRollerVelocity() {
        return bottomRoller.getVelocity();
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