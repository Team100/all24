package org.team100.frc2024.drivetrain;

import org.team100.lib.commands.Subsystem100;
import org.team100.lib.telemetry.SupplierLogger;

import edu.wpi.first.math.geometry.Rotation2d;

public class TankDriveSubsystem extends Subsystem100 {
    private final SupplierLogger m_logger;
    private final TankModuleCollection m_modules;

    public TankDriveSubsystem(
            SupplierLogger parent,
            TankModuleCollection modules) {
        m_logger = parent.child(this);
        m_modules = modules;
    }

    @Override
    public String getGlassName() {
        return "Tank Drive Subsystem";
    }

    @Override
    public void periodic100(double dt) {}

    /**
     * 
     * @param translationSpeed -1 to 1 duty cycle
     * @param rotSpeed -1 to 1 duty cycle
     * 
     */
    public void set(double translationSpeed, double rotSpeed) {
        Rotation2d rot = new Rotation2d(rotSpeed, translationSpeed);
        double power = Math.hypot(translationSpeed, rotSpeed);
        double angle = rot.getRadians();
        if (angle >= 0 && angle < Math.PI/2) {  
            wheelDrive(((Math.PI/4)-angle)/(Math.PI/4), 2, power, angle); 
          } else if (angle >= Math.PI && angle < 3*Math.PI/2) {
            wheelDrive((angle-(5*Math.PI/4))/(Math.PI/4), 2, power, angle);
          } else if (angle >= Math.PI/2 && angle < Math.PI) {
            wheelDrive(((3*Math.PI/4)-angle)/(Math.PI/4), 1, power, angle);
          } else if (angle >= (3*Math.PI/2) && angle < 2*Math.PI) {
            wheelDrive((angle-(7*Math.PI/4))/(Math.PI/4), 1, power, angle);
          }
    }

    public void setRawModules(double... states) {
        m_modules.setDrive(states);
    }

    private void wheelDrive(double angle, int motor, double power, double realangle) {
        if(motor == 2) {
            int neg;
            if (realangle <= Math.PI) {
            neg = 1;
            } else {
              neg = -1;
            }
            setRawModules(neg*power*100,power*(angle*100));
          } else {
            int neg;
            if (realangle >= Math.PI) {
            neg = 1;
            } else {
              neg = -1;
            }
            setRawModules(neg*power*100,power*(angle*100));
          }
              if (realangle == Math.PI) {
            setRawModules(-power*100,-power*100);
        }
    }

    public void stop() {
        m_modules.stop();
    }
}
