package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.armMotion.ArmAngles;
import frc.robot.armMotion.ArmKinematics;

public class ArmSubsystem extends Subsystem{
    public static class Config {
        public double softStop = -0.594938;
        public double kUpperArmLengthM = 0.92;
        public double kLowerArmLengthM = 0.93;
        public double filterTimeConstantS = 0.06; // TODO: tune the time constant
        public double filterPeriodS = 0.02;
        public double safeP = 2.5;
        public double safeI = 0;
        public double safeD = 0;
        public double normalLowerP = 2;
        public double normalLowerI = 0;
        public double normalLowerD = 0.1;
        public double normalUpperP = 2;
        public double normalUpperI = 0;
        public double normalUpperD = 0.05;
        public double tolerance = 0.001;
    }

    public final ArmKinematics kinematics = new ArmKinematics(.93, .92);
    public final double kA = 0.2;
    private final Config m_config = new Config();
    private LinearFilter m_lowerMeasurementFilter;
    private LinearFilter m_upperMeasurementFilter;
    private FRCNEO lowerArmMotor;
    private FRCNEO upperArmMotor;
    private AnalogInput lowerArmInput;
    private AnalogInput upperArmInput;
    private AnalogEncoder lowerArmEncoder;
    private AnalogEncoder upperArmEncoder;

    public ArmSubsystem() {
        m_lowerMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        m_upperMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);

        lowerArmMotor = new FRCNEO.FRCNEOBuilder(4)
                .withInverted(false)
                .withSensorPhase(false)
                .withTimeout(10)
                .withCurrentLimitEnabled(true)
                .withCurrentLimit(8)
                .withPeakOutputForward(0.5)
                .withPeakOutputReverse(-0.5)
                .withNeutralMode(IdleMode.kBrake)
                .build();

        upperArmMotor = new FRCNEO.FRCNEOBuilder(30)
                .withInverted(false)
                .withSensorPhase(false)
                .withTimeout(10)
                .withCurrentLimitEnabled(true)
                .withCurrentLimit(1)
                .withPeakOutputForward(0.5)
                .withPeakOutputReverse(-0.5)
                .withNeutralMode(IdleMode.kBrake)
                .withForwardSoftLimitEnabled(false)
                .build();
        lowerArmInput = new AnalogInput(1);
        lowerArmEncoder = new AnalogEncoder(lowerArmInput);
        upperArmInput = new AnalogInput(0);
        upperArmEncoder = new AnalogEncoder(upperArmInput);
    }

    private double getLowerArm() {
        double x = (lowerArmEncoder.getAbsolutePosition() - 0.861614) * 360;
        return (-1.0 * x) * Math.PI / 180;
    }

    /** Upper arm angle (radians), 0 up, positive forward. */
    private double getUpperArm() {
        double x = (upperArmEncoder.getAbsolutePosition() - 0.266396) * 360;
        return x * Math.PI / 180;
    }

    public ArmAngles getMeasurement() {
        return new ArmAngles(
                m_lowerMeasurementFilter.calculate(getLowerArm()),
                m_upperMeasurementFilter.calculate(getUpperArm()));
    }

    public void set(double u1, double u2) {
        lowerArmMotor.set(u1);
        upperArmMotor.set(u2);
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}
