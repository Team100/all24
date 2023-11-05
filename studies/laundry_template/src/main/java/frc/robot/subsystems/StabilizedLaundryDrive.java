package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.team100.lib.sensors.LSM6DSOX_I2C;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is cut-and-paste from DirectLaundryDrive so we can experiment with
 * stabilization.
 */
public class StabilizedLaundryDrive implements LaundryDrive {
    private final LSM6DSOX_I2C m_gyro;
    private final DoubleSupplier m_xSpeed1_1;
    private final DoubleSupplier m_zSpeed1_1;
    private final PIDController m_controller;
    private final DifferentialDrive m_drive;

    private final DoublePublisher xSpeedPub;
    private final DoublePublisher desiredZSpeedPub;
    private final DoublePublisher measuredZSpeedPub;
    private final DoublePublisher errorZSpeedPub;
    private final DoublePublisher outputZSpeedPub;

    private boolean m_enabled;
    private double i;

    /**
     * @param gyro measures rotation rate
     * @param xSpeed1_1 supplies desired x speed in [-1,1] interval.
     *                  TODO: speed in meters per second.
     * @param zSpeed1_1 supplies desired rotation rate in [-1,1] interval.
     *                  TODO: rotation rate in rad/s
     * @param controller tries to match the measured turn rate to the desired turn rate
     * @param drive     provides "arcade" mode
     */
    public StabilizedLaundryDrive(
            LSM6DSOX_I2C gyro,
            DoubleSupplier xSpeed1_1,
            DoubleSupplier zSpeed1_1,
            PIDController controller,
            DifferentialDrive drive) {
        m_gyro = gyro;
        m_xSpeed1_1 = xSpeed1_1;
        m_zSpeed1_1 = zSpeed1_1;
        m_controller = controller;
        m_drive = drive;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("drive");
        xSpeedPub = table.getDoubleTopic("xSpeed1_1").publish();
        desiredZSpeedPub = table.getDoubleTopic("desiredZSpeedRadS").publish();
        measuredZSpeedPub = table.getDoubleTopic("measuredZSpeedRadS").publish();
        errorZSpeedPub = table.getDoubleTopic("errorZSpeedRadS").publish();
        outputZSpeedPub = table.getDoubleTopic("outputZSpeed1-1").publish();
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
        double xSpeed1_1 = m_xSpeed1_1.getAsDouble();
        xSpeedPub.set(xSpeed1_1);

        // this is a guess about full-scale rotation rate.
        // TODO: tune the full-scale rotation rate.
        // note the extra wide deadband
        double desiredZSpeedRadS = MathUtil.applyDeadband(m_zSpeed1_1.getAsDouble() * 2.0, 0.1);
        desiredZSpeedPub.set(desiredZSpeedRadS);

        double measuredYawRateRadS = m_gyro.getYawRateRadS();
        measuredZSpeedPub.set(measuredYawRateRadS);

        double outputZSpeed1_1 = MathUtil.clamp(m_controller.calculate(measuredYawRateRadS, desiredZSpeedRadS),-1,1);
        errorZSpeedPub.set(m_controller.getPositionError());
        outputZSpeedPub.set(outputZSpeed1_1);

        if (m_enabled) {
            m_drive.arcadeDrive(xSpeed1_1, outputZSpeed1_1, false);
        }
    }
}
