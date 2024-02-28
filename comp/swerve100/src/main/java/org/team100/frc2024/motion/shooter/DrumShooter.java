package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.GravityServo;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.encoder.DutyCycleEncoder100;
import org.team100.lib.encoder.SparkMaxEncoder;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.OutboardVelocityServo;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.simple.Speeding;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.MotorWithEncoder100;
import org.team100.lib.motor.drive.Falcon6DriveMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Direct-drive shooter with top and bottom drums.
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.1m => 0.314 m/turn
 * therefore top speed is around 30 m/s.
 * 
 * Empirically it seems to take a second or so to spin
 * up, so set the acceleration a bit higher than that to start.
 */
public class DrumShooter extends Shooter{
    // TODO: tune the current limit
    /**
     * Muzzle velocity of game piece exiting the shooter.
     * 
     * The shooter should do whatever is necessary to achieve this;
     * a good approximation for a sticky shooter is the surface
     * speed of whatever wheels are contacting the game piece,
     * but there are many factors that affect the relationship in
     * the real world.
     */
    private final String m_name;
    private final VelocityServo<Distance100>  leftRoller;
    private final VelocityServo<Distance100>  rightRoller ;
    private final GravityServo pivotServo;

    private final CANSparkMax pivotMotor;


    private final SpeedingVisualization m_viz;

    private final Telemetry t;

    public final double kLeftRollerVelocity = 30;
    public final double kRightRollerVelocity = 20;


    public DrumShooter(int leftID, int rightID, int pivotID, int feederID, int currentLimit) {
        m_name = Names.name(this);
        
        int shooterCurrentLimit = 40;
        int pivotLimit = 40;
        int feederLimit = 40;

        SysParam shooterParams = SysParam.limitedNeoVelocityServoSystem(1, 0.1, 30, 40, -40);
        SysParam pivotParams = SysParam.neoPositionServoSystem(
            165,
            50,
            50)
            ;        
        SysParam feederParams = SysParam.limitedNeoVelocityServoSystem(1, 0.1, 30, 40, -40);

        t = Telemetry.get();
                
        switch (Identity.instance) {
            case COMP_BOT:
            //TODO tune kV

                MotorWithEncoder100<Distance100> leftMotor = new Falcon6DriveMotor(
                    m_name + "/Left",
                    leftID,
                    false,
                    currentLimit,
                    1,
                    0.1,
                    new PIDConstants(0.4, 0, 0), //0.4
                    new FeedforwardConstants(0.11,0,0,0.9)
                );

                leftRoller = new OutboardVelocityServo<>(m_name, leftMotor, leftMotor);
                        
                MotorWithEncoder100<Distance100> rightMotor = new Falcon6DriveMotor(
                    m_name + "/Riht",
                    rightID,
                    true,
                    currentLimit,
                    1,
                    0.1,
                    new PIDConstants(0.4, 0, 0), //0.4
                    new FeedforwardConstants(0.11,0,0,0.9)
                );

                rightRoller = new OutboardVelocityServo<>(m_name, rightMotor, rightMotor);

                pivotMotor = new CANSparkMax(pivotID, MotorType.kBrushless);
                pivotMotor.setIdleMode(IdleMode.kCoast);
                pivotServo = new GravityServo(
                        pivotMotor,
                        40,
                        m_name + "/Pivot", 
                        pivotParams, 
                        new PIDController(2, 0.0, 0.000), //same
                        new TrapezoidProfile100(8, 8, 0.001),
                        pivotID, 
                        0.02, 
                        0,
                        new DutyCycleEncoder100("SHOOTER PIVOT", 5, 0.284, true),
                        new double[]{0, 45}

                ); //same

                break;
            case BLANK:
            default:
                leftRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Top",
                        shooterParams);
                rightRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Bottom",
                        shooterParams);

                pivotMotor = new CANSparkMax(pivotID, MotorType.kBrushless);

                pivotServo = new GravityServo(
                        pivotMotor,
                        40,
                        m_name + "/Pivot", 
                        pivotParams, 
                        new PIDController(0.07, 0.0, 0.000), //same
                        new TrapezoidProfile100(450, 450, 0.02),
                        pivotID, 
                        0.02, 
                        0,
                        new DutyCycleEncoder100("SHOOTER PIVOT", 5, 0, true),
                        new double[]{0, 45}


                ); //same

        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    @Override
    public void forward() {
        leftRoller.setVelocity(kLeftRollerVelocity);
        rightRoller.setVelocity(kRightRollerVelocity);
    }

    @Override
    public void stop() {
        leftRoller.setDutyCycle(0);
        rightRoller.setDutyCycle(0);
        pivotServo.setDutyCycle(0);
    }

    @Override
    public void reset(){
        pivotServo.reset();
    }

    @Override
    public void rezero(){
        pivotServo.rezero();
    }

    @Override
    public void setAngle(Double goal){

        pivotServo.setPosition(goal);

    }

    @Override
    public void setAngleWithOverride(Double goal, double pivotUp, double pivotDown){

        // if(pivotUp >= 0){
        //     pivotServo.setDutyCycle(pivotUp);
        // } else if(pivotDown >= 0){
        //     pivotServo.setDutyCycle(pivotDown);
        // } else {
        //     pivotServo.setPosition(goal);
        // }

        pivotServo.setPosition(goal);


    }

    public double getAngle(){
        return pivotServo.getPosition();

    }

    @Override
    public void periodic() {
        leftRoller.periodic();
        rightRoller.periodic();
        pivotServo.periodic();
        m_viz.periodic();
 

        
    }

    public void pivotAndRamp(SwerveDriveSubsystem m_drive, double kThreshold){
        if (m_drive.getPose().getX() < kThreshold) {
                forward();
                t.log(Level.DEBUG, m_name, "Angle", ShooterUtil.getAngle(m_drive.getPose().getX()));
                t.log(Level.DEBUG, m_name, "Pose X", m_drive.getPose().getX());

                setAngle(ShooterUtil.getAngle(m_drive.getPose().getX()));
        }
    }

    public boolean readyToShoot(SwerveDriveSubsystem m_drive){
        if (ShooterUtil.getRobotRotationToSpeaker(m_drive.getPose().getTranslation(), 0.25).getDegrees() < 1){
            if(getAngle() - ShooterUtil.getAngle(m_drive.getPose().getX()) < 0.1){
                return true;
            }
        }

        return false;
    }

    public boolean readyToShoot() {
        //TODO get real values here
        return leftRoller.getVelocity() > 30 && rightRoller.getVelocity() > 30;
    }

    public void setDutyCycle(double value) {
        leftRoller.setDutyCycle(value);
        rightRoller.setDutyCycle(value);


    }

    public double getPivotPosition(){
        return pivotServo.getRawPosition();
    }

    public void setPivotPosition(double value){
        pivotServo.setPosition(value);
    }

    public void feed(){
        leftRoller.setDutyCycle(0.3);
        rightRoller.setDutyCycle(0.3);

    }

    
    public double getVelocity() {
        return 0;
    }


    public boolean atVelocitySetpoint(){
        if(leftRoller.getVelocity() >= kLeftRollerVelocity){
            if(rightRoller.getVelocity() >= kRightRollerVelocity){
                return true;
            }
        }

        return false;
    }


   
}