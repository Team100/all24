package org.team100.frc2023.subsystems.arm;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.telemetry.Telemetry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Arm servo. Subsystem runs *all the time*, commands feed it references.
 */
public class ArmSubsystem extends Subsystem implements ArmInterface {
    private static class Noop extends Subsystem implements ArmInterface {

        @Override
        public Subsystem subsystem() {
            return this;
        }

        @Override
        public boolean getCubeMode() {
            return false;
        }

        @Override
        public void setCubeMode(boolean b) {
            //
        }

        @Override
        public void setReference(ArmAngles reference) {
            //
        }

        @Override
        public ArmAngles getMeasurement() {
            return new ArmAngles(0, 0);
        }

        @Override
        public void setControlNormal() {
            //
        }

        @Override
        public void setControlSafe() {
            //
        }

        @Override
        public void close() {
            //
        }
    }

    public static class Factory {
        private final Identity m_identity;

        public Factory(Identity identity) {
            m_identity = identity;
        }

        public ArmInterface get() {
            switch (m_identity) {
                case COMP_BOT:
                    return new ArmSubsystem();
                default:
                    return new Noop();
            }
        }
    }

    public static class Config {
        public double softStop = -0.594938;
        public double kUpperArmLengthM = 0.92;
        public double kLowerArmLengthM = 0.93;
        public double filterTimeConstantS = 0.06; // TODO: tune the time constant
        public double filterPeriodS = 0.02;
        public double safeP = 2.5;
        public double safeI = 0;
        public double safeD = 0;
        public double normalLowerP = 0.3;
        public double normalLowerI = 0;
        public double normalLowerD = 0.1;
        public double normalUpperP = 4;
        public double normalUpperI = 0.2;
        public double normalUpperD = 0.05;
        public double tolerance = 0.001;
    }

    // TODO: do something with this
    public static class ArmState {
        private final State100 m_lower;
        private final State100 m_upper;

        public ArmState(State100 lower, State100 upper) {
            m_lower = lower;
            m_upper = upper;
        }

        public State100 lower() {
            return m_lower;
        }

        public State100 upper() {
            return m_upper;
        }
    }

    private final Config m_config = new Config();

    private final Telemetry t = Telemetry.get();

    /** Kinematics in meters. Coordinates are x up, y forward. */
    private final ArmKinematics m_armKinematicsM;
    private final LinearFilter m_lowerMeasurementFilter;
    private final LinearFilter m_upperMeasurementFilter;
    private final PIDController m_lowerController;
    private final PIDController m_upperController;
    private final CANSparkMax lowerArmMotor;
    private final CANSparkMax upperArmMotor;
    private final AnalogInput lowerArmInput;
    private final AnalogInput upperArmInput;
    private final AnalogEncoder lowerArmEncoder;
    private final AnalogEncoder upperArmEncoder;

    private boolean cubeMode;
    private ArmAngles m_reference;

    private ArmSubsystem() {
        setCubeMode(true);
        m_armKinematicsM = new ArmKinematics(m_config.kLowerArmLengthM, m_config.kUpperArmLengthM);
        m_lowerMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        m_upperMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        m_lowerController = new PIDController(m_config.normalLowerP, m_config.normalLowerI, m_config.normalLowerD);
        m_upperController = new PIDController(m_config.normalUpperP, m_config.normalUpperI, m_config.normalUpperD);
        m_lowerController.setTolerance(m_config.tolerance);
        m_upperController.setTolerance(m_config.tolerance);

		lowerArmMotor = new CANSparkMax(43, MotorType.kBrushless);
		lowerArmMotor.restoreFactoryDefaults();
		lowerArmMotor.setInverted(false);
		lowerArmMotor.setSmartCurrentLimit(40);
		lowerArmMotor.setSecondaryCurrentLimit(40);
		lowerArmMotor.setIdleMode(IdleMode.kBrake);

		upperArmMotor = new CANSparkMax(42, MotorType.kBrushless);
		upperArmMotor.restoreFactoryDefaults();
		upperArmMotor.setInverted(false);
		upperArmMotor.setSmartCurrentLimit(40);
		upperArmMotor.setSecondaryCurrentLimit(40);
		upperArmMotor.setIdleMode(IdleMode.kBrake);        

        lowerArmInput = new AnalogInput(6);
        lowerArmEncoder = new AnalogEncoder(lowerArmInput);
        upperArmInput = new AnalogInput(5);
        upperArmEncoder = new AnalogEncoder(upperArmInput);

        setReference(getMeasurement());
    }

    @Override
    public void periodic() {
        // TODO: add some sort of "enable auto" for this to be safe and not conflict with manual mode
        ArmAngles measurement = getMeasurement();

        t.log("/Arm Subsystem/Upper Arm Measurement Radians", measurement.th2);
        t.log("/Arm Subsystem/Lower Arm Measurement Radians", measurement.th1);
        t.log("/Arm Subsystem/ARM X", m_armKinematicsM.forward(measurement).getX());
        t.log("/Arm Subsystem/ARM Y", m_armKinematicsM.forward(measurement).getY());

        t.log("/Arm Subsystem/Upper Arm Absolute Angle", upperArmEncoder.getAbsolutePosition());
        t.log("/Arm Subsystem/Lower Arm Absolute Angle", lowerArmEncoder.getAbsolutePosition());
        t.log("/Arm Subsystem/Upper Arm Absolute Radians", getUpperArm());
        t.log("/Arm Subsystem/Lower Arm Absolute Radians", getLowerArm());
        t.log("/Arm Subsystem/Upper Arm Absolute Degrees", getUpperArm() * 180 / Math.PI);
        t.log("/Arm Subsystem/Lower Arm Absolute Degrees", getLowerArm() * 180 / Math.PI);

        double u1 = m_lowerController.calculate(measurement.th1, m_reference.th1);
        double u2 = m_upperController.calculate(measurement.th2, m_reference.th2);
        lowerArmMotor.set(soften(u1));
        upperArmMotor.set(u2);

        t.log("/Arm Subsystem/Upper Arm Output amps", lowerArmMotor.getOutputCurrent());
        t.log("/Arm Subsystem/Lower Arm Output amps", upperArmMotor.getOutputCurrent());

    }

    public void setCubeMode(boolean mode) {
        cubeMode = mode;
        t.log("Arm Subsystem/Cube Mode", cubeMode);
    }

    @Override
    public boolean getCubeMode() {
        return cubeMode;
    }

    /**
     * Update the reference to track. This might come from a trajectory sampler or
     * from manual control.
     */
    public void setReference(ArmAngles reference) {
        m_reference = reference;
        t.log("/Arm Subsystem/Upper Angle Setpoint", m_reference.th2);
        t.log("/Arm Subsystem/Lower Angle Setpoint", m_reference.th1);
    }

    /** Measure the arm position, smoothed with single-pole IIR low-pass filter. */
    public ArmAngles getMeasurement() {
        return new ArmAngles(
                m_lowerMeasurementFilter.calculate(getLowerArm()),
                m_upperMeasurementFilter.calculate(getUpperArm()));
    }

    public void setControlNormal() {
        m_lowerController.setPID(m_config.normalLowerP, m_config.normalLowerI, m_config.normalLowerD);
        m_upperController.setPID(m_config.normalUpperP, m_config.normalUpperI, m_config.normalUpperD);
    }

    /**
     * Safe control parameters are gentler, intended to keep from crashing into the
     * safe-position hard-stop.
     */
    public void setControlSafe() {
        m_lowerController.setPID(m_config.safeP, m_config.safeI, m_config.safeD);
        m_upperController.setPID(m_config.safeP, m_config.safeI, m_config.safeD);
    }

    public void close() {
        lowerArmMotor.close();
        upperArmMotor.close();
        lowerArmEncoder.close();
        upperArmEncoder.close();
        lowerArmInput.close();
        upperArmInput.close();
    }

    ///////////////////////////////////////////////////////////////////////////////

    /** Stop the motor at the soft stop. */
    private double soften(double x) {
        if (softBand(x))
            return 0;
        return x;
    }

    /** Should the motor stop? */
    private boolean softBand(double x) {
        if (getLowerArm() <= m_config.softStop && x < 0)
            return false;
        return true;
    }

    /** Lower arm angle (in radians) 0 up, positive forward. */
    private double getLowerArm() {
        double x = (lowerArmEncoder.getAbsolutePosition() - 0.861614) * 360;
        return (-1.0 * x) * Math.PI / 180;
    }

    /** Upper arm angle (radians), 0 up, positive forward. */
    private double getUpperArm() {
        double x = (upperArmEncoder.getAbsolutePosition() - 0.266396) * 360;
        return x * Math.PI / 180;
    }

    @Override
    public Subsystem subsystem() {
        return this;
    }
}