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
 * Direct-drive shooter with right and left wheels.
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.1m => 0.314 m/turn
 * therefore top speed is around 30 m/s.
 * 
 * Empirically it seems to take a second or so to spin
 * up, so set the acceleration a bit higher than that to start.
 */
public class FlywheelShooter extends Shooter implements Speeding {
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
    private final LimitedVelocityServo<Distance100> leftShooter;
    private final LimitedVelocityServo<Distance100> rightShooter;
    private final SpeedingVisualization m_viz;

    public FlywheelShooter(int leftShooterID, int rightShooterID) {
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
                leftShooter = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Left",
                        leftShooterID,
                        false,
                        params);
                rightShooter = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Right",
                        rightShooterID,
                        false,
                        params);
                break;
            case BLANK:
            default:
                leftShooter = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Left",
                        params);

                rightShooter = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Right",
                        params);
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }
    
    public double getFirstRollerVelocity() {
        return leftShooter.getVelocity();
    }
    
    public double getSecondRollerVelocity() {
        return rightShooter.getVelocity();
    }
    @Override
    public void forward() {
        leftShooter.setVelocity(kMuzzleVelocityM_S);
        rightShooter.setVelocity(kMuzzleVelocityM_S);
    }

    @Override
    public void stop() {
        leftShooter.stop();
        rightShooter.stop();
    }

    @Override
    public double getVelocity() {
        return (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2;
    }

    @Override
    public void periodic() {
        leftShooter.periodic();
        rightShooter.periodic();
        m_viz.periodic();
    }
}
