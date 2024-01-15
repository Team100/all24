package org.team100.lib.motion.intake;

import org.team100.lib.motor.turning.NeoTurningMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private NeoTurningMotor bottomRoller;
    private NeoTurningMotor topRoller;
    private final Telemetry t = Telemetry.get();
    private final String m_nameTopRoller;
    private final String m_nameBottomRoller;
    public IntakeSubsystem(String name1, String name2, int canID1, int canID2) {
        m_nameTopRoller=name1;
        m_nameBottomRoller=name2;
            topRoller = new NeoTurningMotor(name1,canID1,true);
            bottomRoller = new NeoTurningMotor(name2,canID2,false);
       }
       //RPS
    public void set(double value) {
        topRoller.setVelocity(value,0);
        bottomRoller.setVelocity(value,0);
    }
    @Override
        public void periodic() {
            t.log(Level.DEBUG, m_nameTopRoller + " Velocity (RPM)", bottomRoller.getVelocity());
            t.log(Level.DEBUG, m_nameBottomRoller + " Velocity (RPM)", topRoller.getVelocity());
        }
}
