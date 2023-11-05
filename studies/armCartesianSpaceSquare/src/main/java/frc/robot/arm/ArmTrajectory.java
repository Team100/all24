package frc.robot.arm;

import frc.robot.ArmSubsystem;
import frc.robot.armMotion.ArmAngles;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmTrajectory extends Command {
    public static class Config {
        public double softStop = -0.594938;
        public double kUpperArmLengthM = 0.92;
        public double kLowerArmLengthM = 0.93;
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
    private boolean t = false;
    private final Config m_config = new Config();
    private final ArmSubsystem m_armSubsystem;
    private final double m_startAngle;
    private final double m_endAngle;
    private final Timer m_timer;
    private final Translation2d m_set;
    private final PIDController m_lowerController;
    private final PIDController m_upperController;
    private final DoublePublisher measurmentX;
    private final DoublePublisher measurmentY;
    private final DoublePublisher setpointUpper;
    private final DoublePublisher setpointLower;

    private Trajectory m_trajectory;

    /**
     * Go to the specified position and optionally oscillate when you get there.
     * Units for angles are degrees
     */
    public ArmTrajectory(ArmSubsystem armSubSystem, Translation2d set, double startAngle, double endAngle) {
        m_set = set;
        m_armSubsystem = armSubSystem;
        m_endAngle = endAngle;
        m_startAngle = startAngle;
        m_timer = new Timer();
        m_lowerController = new PIDController(m_config.normalLowerP, m_config.normalLowerI, m_config.normalLowerD);
        m_upperController = new PIDController(m_config.normalUpperP, m_config.normalUpperI, m_config.normalUpperD);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        measurmentX = inst.getTable("Arm Trajec").getDoubleTopic("measurmentX").publish();
        measurmentY = inst.getTable("Arm Trajec").getDoubleTopic("measurmentY").publish();
        setpointUpper = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Upper").publish();
        setpointLower = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Lower").publish();
    }
    public ArmTrajectory(ArmSubsystem armSubSystem, Translation2d set){
        this(armSubSystem, set, -1, -1);
        t = true;
    }
    @Override
    public void initialize() {
        m_lowerController.setIntegratorRange(1, 1);
        m_upperController.setIntegratorRange(1, 1);
        m_lowerController.setTolerance(m_config.tolerance);
        m_upperController.setTolerance(m_config.tolerance);
        m_timer.restart();
        final TrajectoryConfig trajectoryConfig;
        trajectoryConfig = m_config.normalTrajectory;
        if (t == true) {
        m_trajectory = new ArmTrajectories(trajectoryConfig).makeTrajectory(
                m_armSubsystem.kinematics.forward(m_armSubsystem.getMeasurement()), m_set);
            } else {
                m_trajectory = new ArmTrajectories(trajectoryConfig).onePoint(
                    m_armSubsystem.kinematics.forward(m_armSubsystem.getMeasurement()), m_set, m_startAngle, m_endAngle);
                }

    }

    public void execute() {
        if (m_trajectory == null) {
            return;
        }
        ArmAngles measurement = m_armSubsystem.getMeasurement();
        double currentUpper = measurement.th2;
        double currentLower = measurement.th1;
        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);
        double desiredXPos = desiredState.poseMeters.getX();
        double desiredYPos = desiredState.poseMeters.getY();
        double desiredVecloity = desiredState.velocityMetersPerSecond;
        double desiredAcceleration = desiredState.accelerationMetersPerSecondSq;
        if (desiredState == m_trajectory.sample(10)) {
            desiredAcceleration = 0;
        }
        double theta = desiredState.poseMeters.getRotation().getRadians();
        double desiredXVel = desiredVecloity * Math.cos(theta);
        double desiredYVel = desiredVecloity * Math.cos(Math.PI / 2 - theta);
        double desiredXAccel = desiredAcceleration * Math.cos(theta);
        double desiredYAccel = desiredAcceleration * Math.cos(Math.PI / 2 - theta);
        Translation2d XYVelReference = new Translation2d(desiredXVel, desiredYVel);
        Translation2d XYAccelReference = new Translation2d(desiredXAccel, desiredYAccel);
        XYVelReference = XYVelReference.plus(XYAccelReference.times(m_armSubsystem.kA));
        Translation2d XYPosReference = new Translation2d(desiredXPos, desiredYPos);
        ArmAngles thetaPosReference = m_armSubsystem.kinematics.inverse(XYPosReference);
        ArmAngles thetaVelReference = m_armSubsystem.kinematics.inverseVel(thetaPosReference, XYVelReference);
        double lowerControllerOutput = m_lowerController.calculate(currentLower, thetaPosReference.th1);
        double lowerFeedForward = thetaVelReference.th1 / (Math.PI * 2) * 4;
        double u1 = lowerFeedForward+lowerControllerOutput;
        double upperControllerOutput = m_upperController.calculate(currentUpper, thetaPosReference.th2);
        double upperFeedForward = thetaVelReference.th2 / (Math.PI * 2) * 4;
        double u2 = upperFeedForward+upperControllerOutput;
        m_armSubsystem.set(u1, u2);
        SmartDashboard.putNumber("Lower Encoder: ", currentLower);
        SmartDashboard.putNumber("Lower FF ", lowerFeedForward);
        SmartDashboard.putNumber("Lower Controller Output: ", lowerControllerOutput);
        SmartDashboard.putNumber("Upper FF ", upperFeedForward);
        SmartDashboard.putNumber("Upper Controller Output: ", upperControllerOutput);
        SmartDashboard.putNumber("Lower Ref: ", thetaPosReference.th1);
        SmartDashboard.putNumber("Upper Encoder: ", currentUpper);
        SmartDashboard.putNumber("Upper Ref: ", thetaPosReference.th2);
        SmartDashboard.putNumber("Output Upper: ", u1);
        SmartDashboard.putNumber("Output Lower: ", u2);
        measurmentX.set(currentUpper);
        measurmentY.set(currentLower);
        setpointUpper.set(desiredYPos);
        setpointLower.set(desiredXPos);
    }

    @Override
    public boolean isFinished() {
        double th1 = m_armSubsystem.kinematics.inverse(m_set).th1;
        double th2 = m_armSubsystem.kinematics.inverse(m_set).th2;
        double lowerError = m_armSubsystem.getMeasurement().th1 - th1;
        double upperError = m_armSubsystem.getMeasurement().th2 - th2;
        if (Math.abs(upperError) < 0.02 && Math.abs(lowerError) < 0.02) {
            return true;
        }
        return false;
    }
}
