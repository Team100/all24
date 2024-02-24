package org.team100.frc2024.motion.intake;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.Sensors;
import org.team100.frc2024.RobotState100.IntakeState100;
import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

/**
 * Direct-drive roller intake
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.05m => 0.15 m/turn
 * therefore top speed is around 15 m/s.
 * 
 * This system has very low intertia, so can spin up
 * very fast, but it's fragile: limit accel to avoid stressing it.
 */
public class IntakeRoller extends Intake {
    // TODO: tune the current limit
    private static final int kCurrentLimit = 40;

    /**
     * Surface velocity of whatever is turning in the intake.
     */
    private static final double kIntakeVelocityM_S = 3;
    private static final double kCenteringVelocityM_S = 3;

    private final String m_name;
    private final LimitedVelocityServo<Distance100> intakeRoller;
    // private final LimitedVelocityServo<Distance100> centeringWheels;
    private final PWM centeringWheels;
    private final LimitedVelocityServo<Distance100> superRollers;

    private final SpeedingVisualization m_viz;
    private final Sensors m_sensors;
    private final Telemetry t;

    public IntakeRoller(Sensors sensors, int intakeCAN, int centerCAN, int superCAN) {

        m_name = Names.name(this);

        m_sensors = sensors;


        t = Telemetry.get();

        SysParam rollerParameter = SysParam.limitedNeoVelocityServoSystem(9, 0.05, 15, 10, -10);
        SysParam centeringParameter = SysParam.limitedNeoVelocityServoSystem(1, 0.05, 15, 10, -10);


               
        switch (Identity.instance) {
            case COMP_BOT:
            //TODO tune kV
                intakeRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Intake Roller",
                        intakeCAN,
                        false,
                        kCurrentLimit,
                        rollerParameter,
                        new FeedforwardConstants(0.122,0,0.1,0.065),
                        new PIDConstants(0.0001, 0, 0));
                // centeringWheels = ServoFactory.limitedNeoVelocityServo(
                //         m_name + "/Center Wheels",
                //         centerCAN,
                //         true,
                //         kCurrentLimit,
                //         centeringParameter,
                //         new FeedforwardConstants(0.122,0,0.1,0.065),
                //         new PIDConstants(0.0001, 0, 0));

                centeringWheels = new PWM(0);
                superRollers = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Super Roller",
                        superCAN,
                        true,
                        kCurrentLimit,
                        rollerParameter,
                        new FeedforwardConstants(0.122,0,0.1,0.065),
                        new PIDConstants(0.0001, 0, 0));
                break;
            case BLANK:
            default:
                intakeRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Intake Roller",
                        rollerParameter);
                // centeringWheels = ServoFactory.limitedSimulatedVelocityServo(
                //         m_name + "/Center Wheels",
                //         centeringParameter);
                centeringWheels = new PWM(0);

                superRollers = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Center Wheels",
                        rollerParameter);
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    @Override
    //All you have to do is set the state once and it handles the rest. No need for commands 
    public void intakeSmart() {
        if(!m_sensors.getFeederSensor()){
            System.out.println("STOPING INAKE" + Timer.getFPGATimestamp());
            intakeRoller.setVelocity(0);
            centeringWheels.setSpeed(0);
            superRollers.setVelocity(0);
            RobotState100.changeIntakeState(IntakeState100.STOP);
        } else {
            intakeRoller.setVelocity(kIntakeVelocityM_S);
            centeringWheels.setSpeed(0.8);
            superRollers.setVelocity(kIntakeVelocityM_S);
        }
        

    }

    @Override
    public void intake(){
        // System.out.println("INTAKING");
        // intakeRoller.setVelocity(kIntakeVelocityM_S);
        // centeringWheels.setVelocity(kCenteringVelocityM_S);
        // superRollers.setVelocity(kIntakeVelocityM_S);
        // m_feeder.feed(IntakeRoller.class);
        centeringWheels.setSpeed(0.8);

        intakeRoller.setDutyCycle(0.8);
        superRollers.setDutyCycle(0.8);
        // centeringWheels.setDutyCycle(0.5);
        // centeringWheels.setSpeed(0.8);

    }

    @Override
    public void outtake() {
        intakeRoller.setVelocity(-kIntakeVelocityM_S);
        // centeringWheels.setVelocity(-kCenteringVelocityM_S);
        superRollers.setVelocity(-kIntakeVelocityM_S);
    }

    @Override
    public void stop() {
        // System.out.println("STOPPPP");
        intakeRoller.setDutyCycle(0);
        centeringWheels.setSpeed(0);
        superRollers.setDutyCycle(0);

    }

    @Override
    public void periodic() {
        if(RobotState100.getIntakeState() == IntakeState100.INTAKE){
            t.log(Level.DEBUG, m_name, "STATE", 2);
        } else if(RobotState100.getIntakeState() == IntakeState100.OUTTAKE){
            t.log(Level.DEBUG, m_name, "STATE", 1);
        }else if(RobotState100.getIntakeState() == IntakeState100.STOP){
            t.log(Level.DEBUG, m_name, "STATE", 0);
        }   

        // System.out.println(m_sensors.getFeederSensor());

        intakeRoller.periodic();
        // centeringWheels.periodic();

        superRollers.periodic();
        m_viz.periodic();
    }

    @Override
    public double getVelocity() {
        return intakeRoller.getVelocity();
    }

    
}
