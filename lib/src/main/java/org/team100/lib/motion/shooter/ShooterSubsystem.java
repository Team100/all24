package org.team100.lib.motion.shooter;

import org.team100.lib.motor.turning.NeoTurningMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private NeoTurningMotor leftShooter;
    private NeoTurningMotor rightShooter;
    private final Telemetry t = Telemetry.get();
    private final String m_nameLeftShooter;
    private final String m_nameRightShooter;
        public ShooterSubsystem(String name1, String name2, int canID1,int canID2) {
            m_nameLeftShooter = name1;
            m_nameRightShooter = name2;
            leftShooter = new NeoTurningMotor(name1, canID1,true);
            rightShooter = new NeoTurningMotor(name2, canID2,false);

        }
        //RPS
        public void set(double value) {
            leftShooter.setVelocity(value,0);
            rightShooter.setVelocity(value,0);
        }

        @Override
        public void periodic() {
            t.log(Level.DEBUG, m_nameLeftShooter + " Velocity (RPM)", leftShooter.getVelocity());
            t.log(Level.DEBUG, m_nameRightShooter + " Velocity (RPM)", rightShooter.getVelocity());
        }
        
}
