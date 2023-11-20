package org.team100.lib.motion.arm;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.FRCNEO;
import frc.robot.subsystems.FRCNEO.FRCNEOBuilder;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class ArmSubsystem extends Subsystem {
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
        public double oscillatorFrequencyHz = 2;
        /** amplitude (each way) of oscillation in encoder units */
        public double oscillatorScale = 0.025;
        /** start oscillating when this close to the target. */
        public double oscillatorZone = 0.1;
        public TrajectoryConfig safeTrajectory = new TrajectoryConfig(9, 1.5);
        public TrajectoryConfig normalTrajectory = new TrajectoryConfig(1, 1);
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
    private ArmAngles lastref;
    private final PIDController m_lowerPosController;
    private final PIDController m_upperPosController;
    private final PIDController m_lowerVelController;
    private final PIDController m_upperVelController;
    private Trajectory m_trajectory;
    private Translation2d m_end;
    private Timer m_timer;

    public ArmSubsystem() {
        m_lowerPosController = new PIDController(m_config.normalLowerP, m_config.normalLowerI, m_config.normalLowerD);
        m_upperPosController = new PIDController(m_config.normalUpperP, m_config.normalUpperI, m_config.normalUpperD);
        m_lowerVelController = new PIDController(.1,0, 0);
        m_upperVelController = new PIDController(.1,0, 0);
        m_lowerPosController.setIntegratorRange(1, 1);
        m_upperPosController.setIntegratorRange(1, 1);
        m_lowerPosController.setTolerance(m_config.tolerance);
        m_upperPosController.setTolerance(m_config.tolerance);
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
        lastref = this.getMeasurement();
    }

    /** Lower arm angle (radians), 0 up, positive forward. */
    private double getLowerArm() {
        double encoderZero = 0.861614;
        double x = (lowerArmEncoder.getAbsolutePosition() - encoderZero) * 360;
        return (-1.0 * x) * Math.PI / 180;
    }

    @Override
    public void periodic() {
        if (m_trajectory != null) {
        ArmAngles measurement = this.getMeasurement();
        double currentUpper = measurement.th2;
        double currentLower = measurement.th1;
        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);
        double desiredXPos = desiredState.poseMeters.getX();
        double desiredYPos = desiredState.poseMeters.getY();
        double desiredVecloity = desiredState.velocityMetersPerSecond;
        double desiredAcceleration = desiredState.accelerationMetersPerSecondSq;
        if (desiredState == m_trajectory.sample(100)) {
            desiredAcceleration = 0;
        }
        double theta = desiredState.poseMeters.getRotation().getRadians();
        double desiredXVel = desiredVecloity * Math.cos(theta);
        double desiredYVel = desiredVecloity * Math.cos(Math.PI / 2 - theta);
        double desiredXAccel = desiredAcceleration * Math.cos(theta);
        double desiredYAccel = desiredAcceleration * Math.cos(Math.PI / 2 - theta);
        Translation2d XYVelReference = new Translation2d(desiredXVel, desiredYVel);
        Translation2d XYAccelReference = new Translation2d(desiredXAccel, desiredYAccel);
        XYVelReference = XYVelReference.plus(XYAccelReference.times(this.kA));
        Translation2d XYPosReference = new Translation2d(desiredXPos, desiredYPos);
        ArmAngles thetaPosReference = this.kinematics.inverse(XYPosReference);
        ArmAngles thetaVelReference = this.kinematics.inverseVel(thetaPosReference, XYVelReference);
        double lowerPosControllerOutput = m_lowerPosController.calculate(currentLower, thetaPosReference.th1);
        double lowerVelControllerOutput = m_lowerVelController.calculate(this.getVel().th1, thetaVelReference.th1);
        double rotsPerSecToVoltsPerSec = 4;
        double lowerFeedForward = thetaVelReference.th1 / (Math.PI * 2) * rotsPerSecToVoltsPerSec;
        double u1 = lowerFeedForward+lowerPosControllerOutput+lowerVelControllerOutput;
        double upperPosControllerOutput = m_upperPosController.calculate(currentUpper, thetaPosReference.th2);
        double upperVelControllerOutput = m_upperVelController.calculate(this.getVel().th2, thetaVelReference.th2);
        double upperFeedForward = thetaVelReference.th2 / (Math.PI * 2) * rotsPerSecToVoltsPerSec;
        double u2 = upperFeedForward+upperPosControllerOutput+upperVelControllerOutput;
        this.set(u1, u2);
        SmartDashboard.putNumber("Lower FF ", lowerFeedForward);
        SmartDashboard.putNumber("Lower Controller Output: ", lowerPosControllerOutput);
        SmartDashboard.putNumber("Upper FF ", upperFeedForward);
        SmartDashboard.putNumber("Upper Controller Output: ", upperPosControllerOutput);
        SmartDashboard.putNumber("Lower Ref: ", thetaPosReference.th1);
        SmartDashboard.putNumber("Upper Ref: ", thetaPosReference.th2);
        SmartDashboard.putNumber("Output Upper: ", u1);
        SmartDashboard.putNumber("Output Lower: ", u2);
        }
        SmartDashboard.putNumber("Lower Encoder Pos: ",  this.getMeasurement().th1);
        SmartDashboard.putNumber("Lower Encoder Vel: ",  this.getVel().th1);
        SmartDashboard.putNumber("Upper Encoder Pos: ", this.getMeasurement().th2);
        SmartDashboard.putNumber("Upper Encoder Vel: ",  this.getVel().th2);
    }

    public void setTrajectory(Timer timer, Translation2d end) {
        m_end = end;
        m_timer = timer;
        final TrajectoryConfig trajectoryConfig;
        trajectoryConfig = m_config.normalTrajectory;
        m_trajectory = new ArmTrajectories(trajectoryConfig).makeTrajectory(
                    this.kinematics.forward(this.getMeasurement()), end);
    }

    public void setTrajectory(Timer timer, Translation2d end, double startAngle, double EndAngle) {
        m_end = end;
        m_timer = timer;
        final TrajectoryConfig trajectoryConfig;
        trajectoryConfig = m_config.normalTrajectory;
        m_trajectory = new ArmTrajectories(trajectoryConfig).onePoint(
                    this.kinematics.forward(this.getMeasurement()), end,startAngle,EndAngle);
    }

    public ArmAngles getTrajectoryError() {
        if (m_trajectory == null) {
            System.out.println("There is no trajectory currently given");
            return null;
        }
        double th1 = this.kinematics.inverse(m_end).th1;
        double th2 = this.kinematics.inverse(m_end).th2;
        double lowerError = this.getMeasurement().th1 - th1;
        double upperError = this.getMeasurement().th2 - th2;
        return new ArmAngles(lowerError, upperError);
    }

    /** Upper arm angle (radians), 0 up, positive forward. */
    private double getUpperArmAngleRadians() {
        double encoderZero = 0.266396;
        double x = (upperArmEncoder.getAbsolutePosition() - encoderZero) * 360;
        return x * Math.PI / 180;
    }

    /** Arm angles (radians), 0 up, positive forward. */
    public ArmAngles getMeasurement() {
        return new ArmAngles(
                m_lowerMeasurementFilter.calculate(getLowerArm()),
                m_upperMeasurementFilter.calculate(getUpperArmAngleRadians()));
    }

    public ArmAngles getVel() {
        double th1 = lastref.th1 - getMeasurement().th1;
        double th2 = lastref.th2 - getMeasurement().th2;
        lastref = this.getMeasurement();
        return new ArmAngles(th1*50, th2*50);
    }

    public void set(double u1, double u2) {
        lowerArmMotor.set(u1);
        upperArmMotor.set(u2);
    }

    public void endTrajectory() {
        this.set(0,0);
        m_trajectory = null;
    }
}
