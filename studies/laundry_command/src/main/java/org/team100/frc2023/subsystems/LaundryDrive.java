package org.team100.frc2023.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LaundryDrive extends Subsystem {
    private final DifferentialDrive m_drive;
    private final DoublePublisher xSpeedPub;
    private final DoublePublisher zSpeedPub;
    private double m_xSpeed1_1;
    private double m_zSpeed1_1;
    private boolean m_enabled;

    /** Operates the differential drive. */
    public LaundryDrive(DifferentialDrive drive) {
        m_drive = drive;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("drive");
        xSpeedPub = table.getDoubleTopic("xSpeed1_1").publish();
        zSpeedPub = table.getDoubleTopic("zSpeed1-1").publish();
    }

    public void setSpeeds(double xSpeed1_1, double zSpeed1_1) {
        m_xSpeed1_1 = xSpeed1_1;
        m_zSpeed1_1 = zSpeed1_1;
        xSpeedPub.set(xSpeed1_1);
        zSpeedPub.set(zSpeed1_1);
    }

    public void enable() {
        m_xSpeed1_1 = 0;
        m_zSpeed1_1 = 0;
        m_enabled = true;
    }

    public void disable() {
        m_enabled = false;
        m_xSpeed1_1 = 0;
        m_zSpeed1_1 = 0;
        m_drive.stopMotor();
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            m_drive.arcadeDrive(m_xSpeed1_1, m_zSpeed1_1, false);
        } else {
            m_drive.stopMotor();
        }
    }
}
