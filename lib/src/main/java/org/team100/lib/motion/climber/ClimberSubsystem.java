package org.team100.lib.motion.climber;

import org.team100.lib.motor.turning.NeoTurningMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final Telemetry t = Telemetry.get();
    private final String m_nameLeftClimber;
    private final String m_nameRightClimber;
    private NeoTurningMotor leftClimber;
    private NeoTurningMotor rightClimber;
        public ClimberSubsystem(String name1, String name2, int canID1,int canID2) {
            m_nameLeftClimber = name1;
            m_nameRightClimber = name2;
            leftClimber = new NeoTurningMotor(name1, canID1,true);
            rightClimber = new NeoTurningMotor(name2, canID2,true);

        }
        //RPS
        public void set(double value) {
            leftClimber.setVelocity(value,0);
            rightClimber.setVelocity(value,0);
        }
        @Override
        public void periodic() {
            t.log(Level.DEBUG, m_nameLeftClimber + " Velocity (RPM)", leftClimber.getVelocity());
            t.log(Level.DEBUG, m_nameRightClimber + " Velocity (RPM)", rightClimber.getVelocity());
        }
}
