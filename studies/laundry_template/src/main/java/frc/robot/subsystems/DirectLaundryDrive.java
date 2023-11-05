package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DirectLaundryDrive implements LaundryDrive {
    private final DoubleSupplier m_xSpeed1_1;
    private final DoubleSupplier m_zSpeed1_1;
    private final DifferentialDrive m_drive;

    private final DoublePublisher xSpeedPub;
    private final DoublePublisher zSpeedPub;

    private boolean m_enabled;
    private double i;

    /**
     * @param xSpeed1_1 supplies desired x speed in [-1,1] interval.
     *                  TODO: speed in meters per second.
     * @param zSpeed1_1 supplies desired rotation rate in [-1,1] interval.
     *                  TODO: rotation rate in rad/s
     * @param drive     provides "arcade" mode
     */
    public DirectLaundryDrive(
            DoubleSupplier xSpeed1_1,
            DoubleSupplier zSpeed1_1,
            DifferentialDrive drive) {
        m_xSpeed1_1 = xSpeed1_1;
        m_zSpeed1_1 = zSpeed1_1;
        m_drive = drive;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("drive");
        xSpeedPub = table.getDoubleTopic("xSpeed1_1").publish();
        zSpeedPub = table.getDoubleTopic("zSpeed1-1").publish();
    }

    @Override
    public void enable() {
        m_enabled = true;
    }

    @Override
    public void disable() {
        m_enabled = false;
        m_drive.stopMotor();
    }

    @Override
    public void autonomousInit() {
        i = 0;
    }

    @Override
    public void autonomousPeriodic() {
        if (i < 115) {
            m_drive.arcadeDrive(-0.4, -0.05, false);
            i++;
        }
    }

    @Override
    public void teleopPeriodic() {
        double xSpeed1_1 = m_xSpeed1_1.getAsDouble();
        xSpeedPub.set(xSpeed1_1);

        double zSpeed1_1 = m_zSpeed1_1.getAsDouble();
        zSpeedPub.set(zSpeed1_1);

        if (m_enabled) {
            m_drive.arcadeDrive(xSpeed1_1, zSpeed1_1, false);
        }
    }
}
