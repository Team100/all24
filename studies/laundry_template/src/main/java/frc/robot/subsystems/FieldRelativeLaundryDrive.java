package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.team100.lib.sensors.AHRS;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class FieldRelativeLaundryDrive implements LaundryDrive {
    public static class Control {
        public final double x;
        public final double y;

        public Control(double x, double y) {
            this.x = x;
            this.y = y;
        }

        /**
         * x and y refer to field coords from driver perspective, so x is straight
         * ahead, y is to the left
         */
        public static Control rotate(double headingRad, double x, double y) {
            double cos = Math.cos(headingRad);
            double sin = Math.sin(headingRad);
            double fwd = x * cos + y * sin;
            double rot = -x * sin + y * cos;
            return new Control(fwd, rot);
        }
    }

    private final AHRS m_ahrs;
    private final BooleanSupplier m_reset;
    private final DoubleSupplier m_xSpeed1_1;
    private final DoubleSupplier m_ySpeed1_1;
    private final DifferentialDrive m_drive;

    private final DoublePublisher xSpeedPub;
    private final DoublePublisher zSpeedPub;

    private boolean m_enabled;
    private double i;

    /**
     * @param ahrs      heading supplier
     * @param reset     reset button supplier
     * @param xSpeed1_1 supplies desired field x speed in [-1,1] interval.
     *                  TODO: speed in meters per second.
     * @param ySpeed1_1 supplies desired field y rate in [-1,1] interval.
     *                  TODO: rotation rate in rad/s
     * @param drive     provides "arcade" mode
     */
    public FieldRelativeLaundryDrive(
            AHRS ahrs,
            BooleanSupplier reset,
            DoubleSupplier xSpeed1_1,
            DoubleSupplier ySpeed1_1,
            DifferentialDrive drive) {
        m_ahrs = ahrs;
        m_reset = reset;
        m_xSpeed1_1 = xSpeed1_1;
        m_ySpeed1_1 = ySpeed1_1;
        m_drive = drive;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("FieldRelativeDrive");
        xSpeedPub = table.getDoubleTopic("xSpeed1_1").publish();
        zSpeedPub = table.getDoubleTopic("zSpeed1_1").publish();
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
        if (i < 100) {
            m_drive.arcadeDrive(-1, 0, false);
            i++;
        }
    }

    @Override
    public void teleopPeriodic() {
        if (m_reset.getAsBoolean()) {
            m_ahrs.reset();
        }
        m_ahrs.update();
        double xSpeed1_1 = m_xSpeed1_1.getAsDouble();
        double ySpeed1_1 = m_ySpeed1_1.getAsDouble();

        Control control = Control.rotate(m_ahrs.getHeadingNWURad(), xSpeed1_1, ySpeed1_1);
        xSpeedPub.set(control.x);
        zSpeedPub.set(control.y);

        if (m_enabled) {
            m_drive.arcadeDrive(control.x, control.y, false);
        }
    }
}
