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

public class ArmTrajectoryCommand extends Command {
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
        public double normalUpperP = 1.5;
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
    private final PIDController m_lowerPosController;
    private final PIDController m_upperPosController;
    private final PIDController m_lowerVelController;
    private final PIDController m_upperVelController;
    private final DoublePublisher measurmentX;
    private final DoublePublisher measurmentY;
    private final DoublePublisher setpointUpper;
    private final DoublePublisher setpointLower;

    private Trajectory m_trajectory;

    /**
     * Go to the specified position and optionally oscillate when you get there.
     * Units for angles are degrees
     */
    public ArmTrajectoryCommand(ArmSubsystem armSubSystem, Translation2d set, double startAngle, double endAngle) {
        m_set = set;
        m_armSubsystem = armSubSystem;
        m_endAngle = endAngle;
        m_startAngle = startAngle;
        m_timer = new Timer();
        m_lowerPosController = new PIDController(m_config.normalLowerP, m_config.normalLowerI, m_config.normalLowerD);
        m_upperPosController = new PIDController(m_config.normalUpperP, m_config.normalUpperI, m_config.normalUpperD);
        m_lowerVelController = new PIDController(.1,0, 0);
        m_upperVelController = new PIDController(.1,0, 0);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        measurmentX = inst.getTable("Arm Trajec").getDoubleTopic("measurmentX").publish();
        measurmentY = inst.getTable("Arm Trajec").getDoubleTopic("measurmentY").publish();
        setpointUpper = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Upper").publish();
        setpointLower = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Lower").publish();
    }

    public ArmTrajectoryCommand(ArmSubsystem armSubSystem, Translation2d set) {
        this(armSubSystem, set, -1, -1);
        t = true;
    }

    @Override
    public void initialize() {
        m_lowerPosController.setIntegratorRange(1, 1);
        m_upperPosController.setIntegratorRange(1, 1);
        m_lowerPosController.setTolerance(m_config.tolerance);
        m_upperPosController.setTolerance(m_config.tolerance);
        m_timer.restart();
        final TrajectoryConfig trajectoryConfig;
        trajectoryConfig = m_config.normalTrajectory;
        if (t == true) {
            m_trajectory = new ArmTrajectories(trajectoryConfig).makeTrajectory(
                    m_armSubsystem.kinematics.forward(m_armSubsystem.getMeasurement()), m_set);
        } else {
            if (m_startAngle == -1 && m_endAngle == -1) {
                System.out.println("POSSIBLE ERROR");
            }
            m_trajectory = new ArmTrajectories(trajectoryConfig).onePoint(
                    m_armSubsystem.kinematics.forward(m_armSubsystem.getMeasurement()), m_set, m_startAngle,
                    m_endAngle);
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
        if (desiredState == m_trajectory.sample(100)) {
            desiredAcceleration = 0;
        }
        System.out.println(desiredAcceleration);
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
        double lowerPosControllerOutput = m_lowerPosController.calculate(currentLower, thetaPosReference.th1);
        double lowerVelControllerOutput = m_lowerVelController.calculate(m_armSubsystem.getVel().th1, thetaVelReference.th1);
        double rotsPerSecToVoltsPerSec = 4;
        double lowerFeedForward = thetaVelReference.th1 / (Math.PI * 2) * rotsPerSecToVoltsPerSec;
        double u1 = lowerFeedForward+lowerPosControllerOutput+lowerVelControllerOutput;
        double upperPosControllerOutput = m_upperPosController.calculate(currentUpper, thetaPosReference.th2);
        double upperVelControllerOutput = m_upperVelController.calculate(m_armSubsystem.getVel().th2, thetaVelReference.th2);
        double upperFeedForward = thetaVelReference.th2 / (Math.PI * 2) * rotsPerSecToVoltsPerSec;
        double u2 = upperFeedForward+upperPosControllerOutput+upperVelControllerOutput;
        m_armSubsystem.set(u1, u2);
        SmartDashboard.putNumber("Lower FF ", lowerFeedForward);
        SmartDashboard.putNumber("Lower Controller Output: ", lowerPosControllerOutput);
        SmartDashboard.putNumber("Upper FF ", upperFeedForward);
        SmartDashboard.putNumber("Upper Controller Output: ", upperPosControllerOutput);
        SmartDashboard.putNumber("Lower Ref: ", thetaPosReference.th1);
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
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("FINISHEDDDDDDDDDD");
        m_armSubsystem.set(0, 0);
    }
}