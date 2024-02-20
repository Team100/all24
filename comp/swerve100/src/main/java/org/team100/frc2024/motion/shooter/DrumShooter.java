package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.GravityServo;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.simple.Speeding;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

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
    private static final double kMuzzleVelocityM_S = 30;
    private final String m_name;
    private final FeederSubsystem m_feeder;
    private final LimitedVelocityServo<Distance100> leftRoller;
    private final LimitedVelocityServo<Distance100> rightRoller;
    private final GravityServo pivotMotor;

    private final SpeedingVisualization m_viz;

    private final Telemetry t;


    public DrumShooter(int leftID, int rightID, int pivotID, int feederID, int currentLimit, FeederSubsystem feeder) {
        m_name = Names.name(this);
        
        int shooterCurrentLimit = 40;
        int pivotLimit = 40;
        int feederLimit = 40;

        SysParam shooterParams = SysParam.limitedNeoVelocityServoSystem(1, 0.1, 30, 40, -40);
        SysParam pivotParams = SysParam.neoPositionServoSystem(
            75,
            30,
            30)
            ;        
        SysParam feederParams = SysParam.limitedNeoVelocityServoSystem(1, 0.1, 30, 40, -40);

        t = Telemetry.get();
                
        switch (Identity.instance) {
            case COMP_BOT:
            //TODO tune kV
                leftRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Left",
                        leftID,
                        false,
                        shooterCurrentLimit,
                        shooterParams,
                        new FeedforwardConstants(0.122,0,0.1,0.065),
                        new PIDConstants(0.0001, 0, 0));
                rightRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Right",
                        rightID,
                        true,
                        shooterCurrentLimit,
                        shooterParams,
                        new FeedforwardConstants(0.122,0,0.1,0.065),
                        new PIDConstants(0.0001, 0, 0));

                pivotMotor = new GravityServo(
                        m_name + "/Pivot", 
                        pivotParams, 
                        new PIDController(0.01, 0, 0), //same
                        new TrapezoidProfile100(pivotParams.maxVelM_S(), pivotParams.maxAccelM_S2(), 0.05),
                        pivotID, 
                        0.02, 
                        0

                ); //same

                m_feeder = feeder;
                break;
            case BLANK:
            default:
                leftRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Top",
                        shooterParams);
                rightRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Bottom",
                        shooterParams);

                pivotMotor = new GravityServo(
                        m_name + "/Pivot", 
                        pivotParams, 
                        new PIDController(1, 0, 0),
                        new TrapezoidProfile100(pivotParams.maxVelM_S(), pivotParams.maxAccelM_S2(), 0.05),
                        pivotID, 
                        0.02, 
                        -0.06
                ); 

                m_feeder = feeder;
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    @Override
    public void forward() {
        // leftRoller.setVelocity(kMuzzleVelocityM_S);
        // rightRoller.setVelocity(kMuzzleVelocityM_S);
    }

    @Override
    public void feed(){
        // if(readyToShoot()){
            // feedRoller.setVelocity(10);
            m_feeder.feed(DrumShooter.class);

        // }

    };

    @Override
    public void stop() {
        // leftRoller.stop();
        // rightRoller.stop();
        // pivotMotor.stop();
        // m_feeder.stop(DrumShooter.class);
    }

    @Override
    public void reset(){
        pivotMotor.reset();
    }

    @Override
    public void setAngle(Double goal){
        pivotMotor.setPosition(goal);

    }

    public double getAngle(){
        return pivotMotor.getPosition();

    }

    @Override
    public void periodic() {
        leftRoller.periodic();
        rightRoller.periodic();
        pivotMotor.periodic();
        m_viz.periodic();

        // leftRoller.setDutyCycle(-0.1);
        // rightRoller.setDutyCycle(-0.1);

        
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

    public double getVelocity() {
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                return (leftRoller.getVelocity() + rightRoller.getVelocity()) / 2;
            default:
                return 15;
        }
    }
}