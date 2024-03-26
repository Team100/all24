package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.motion.GravityServo;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.encoder.DutyCycleEncoder100;
import org.team100.lib.motion.components.OutboardVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.motor.MotorWithEncoder100;
import org.team100.lib.motor.drive.Falcon6DriveMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    private final DutyCycleEncoder100 m_encoder;


    private final CANSparkMax pivotMotor;


    private final SpeedingVisualization m_viz;

    private final Telemetry t;

    public final double kLeftRollerVelocity = 20;
    public final double kRightRollerVelocity = 20;


    public DrumShooter(int leftID, int rightID, int pivotID, int currentLimit) {
        m_name = Names.name(this);
        m_encoder = new DutyCycleEncoder100("SHOOTER PIVOT", 0, 0.5087535877188397 , false);
        // m_encoder.reset();
        int shooterCurrentLimit = 40;
        int pivotLimit = 40;
        int feederLimit = 40;

        SysParam shooterParams = SysParam.limitedNeoVelocityServoSystem(1, 0.1, 30, 40, -40);
        SysParam pivotParams = SysParam.neoPositionServoSystem(
            165,
            300,
            300)
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

                pivotMotor = new CANSparkMax(27, MotorType.kBrushless);
                pivotMotor.setIdleMode(IdleMode.kCoast);
                pivotServo = new GravityServo(
                        pivotMotor,
                        40,
                        m_name + "/Pivot", 
                        pivotParams, 
                        new PIDController(4.5, 0.0, 0.000), //same
                        new TrapezoidProfile100(8, 8, 0.001),
                        pivotID, 
                        0.02, 
                        0,
                        m_encoder,
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
                        10,
                        m_name + "/Pivot", 
                        pivotParams, 
                        new PIDController(0.07, 0.0, 0.000), //same
                        new TrapezoidProfile100(450, 450, 0.02),
                        pivotID, 
                        0.02, 
                        0,
                        m_encoder,
                        new double[]{0, 45}


                ); //same

        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    @Override
    public void forward() {
        // leftRoller.setVelocity(kLeftRollerVelocity);
        // rightRoller.setVelocity(kRightRollerVelocity);

        leftRoller.setVelocity(2);
        rightRoller.setVelocity(2);
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
        // pivotServo.rezero();
    }

    @Override
    public void setAngle(Double goal){

        pivotServo.setPosition(goal);
        // System.out.println("SETTTING");
        // pivotServo.setDutyCycle(0.1);

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

        // pivotServo.setPosition(goal);


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

        // pivotServo.setDutyCycle(0.1);


        // System.out.println("GET" + m_encoder.m_encoder.get());
        // System.out.println("AHHHHH");
        // System.out.println("Absolute" + pivotServo.getPosition());
        // System.out.println("POSITION OFFSET" + m_encoder.m_encoder.getPositionOffset());
        // System.out.println("DISTANCE PER" + m_encoder.m_encoder.getDistancePerRotation());

        // System.out.println("FREEQ" + m_encoder.m_encoder.getFrequency());
 

        
    }

   

    public void pivotAndRamp(SwerveDriveSubsystem m_drive, double kThreshold){
        if (m_drive.getPose().getX() < kThreshold) {
                forward();
                t.log(Level.DEBUG, m_name, "Angle", ShooterUtil.getAngle(m_drive.getPose().getX()));
                t.log(Level.DEBUG, m_name, "Pose X", m_drive.getPose().getX());

                setAngle(ShooterUtil.getAngle(m_drive.getPose().getX()));
        }
    }

    public boolean readyToShoot(Alliance alliance, SwerveDriveSubsystem m_drive){
        if (ShooterUtil.getRobotRotationToSpeaker(alliance, m_drive.getPose().getTranslation(), 0.25).getDegrees() < 1){
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

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public boolean atVelocitySetpoint(){
        if( Math.abs(leftRoller.getVelocity() - kLeftRollerVelocity) < 10){
            if( Math.abs(rightRoller.getVelocity() - kRightRollerVelocity) < 10){
                return true;
            }
        }

        return false;
    }

    @Override
    public boolean atVelocitySetpoint(boolean bool){

        if(bool){
            if( leftRoller.getVelocity() > (kLeftRollerVelocity/2)){
                if(rightRoller.getVelocity() > (kRightRollerVelocity/2)){
                    return true;
                }
            }
        } else{
            if( Math.abs(leftRoller.getVelocity() - kLeftRollerVelocity) < 0.5){
                if( Math.abs(rightRoller.getVelocity() - kRightRollerVelocity) < 0.5){
                    return true;
                }
            }
        }

        return false;

    }
}