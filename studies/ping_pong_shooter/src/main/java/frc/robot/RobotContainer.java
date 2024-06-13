package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // WHEELS

    private static final double kP = 0.1;
    private static final double kD = 0.1;
    private static final double WHEEL_MAX_RPM = 1500;
    private static final double WHEEL_FF = 2.7;

    // gear ratio for pololu 4789 is 15.24884259
    // for magnet 2599 a quadrature encoder yields 3 four-phase cycles per
    // revolution so ticks per rev is 45.74652778
    private static final double TICKS_PER_TURN = 45.74652778;

    // FEED
    private static final int FEED_DURATION = 7;
    private static final double FEED_SPEED = 1.0;

    private enum Feedstate {
        OFF, REVERSING, ADVANCING
    }

    // ELEVATION
    private static final double ELEVATION_SPEED = 0.01;

    private final Encoder m_wheel_encoder1;
    private final Encoder m_wheel_encoder2;

    private final Feed m_feed;

    private boolean m_advance_feed = false;
    private int m_state_counter = 0;
    private Feedstate m_feedstate = Feedstate.OFF;

    private final Servo m_elevation;

    private final XboxController m_controller;

    // LATER: calibrate elevation
    private double m_current_elevation = 0.5;

    private final PIDController m_wheel_controller1 = new PIDController(kP, 0, kD);
    private final PIDController m_wheel_controller2 = new PIDController(kP, 0, kD);

    private final Wheel m_wheel1 = new Wheel("wheel1", 8);
    private final Wheel m_wheel2 = new Wheel("wheel2", 9);

    ElevationSubsystem m_elev = new ElevationSubsystem();

    public RobotContainer() {
        m_wheel_encoder1 = new Encoder(6, 7, false, EncodingType.k1X);
        m_wheel_encoder1.setDistancePerPulse(60 / TICKS_PER_TURN); // read RPM

        m_wheel_encoder2 = new Encoder(8, 9, false, EncodingType.k1X);
        m_wheel_encoder2.setDistancePerPulse(60 / TICKS_PER_TURN);

        m_feed = new Feed("feed", 7);

        m_elevation = new Servo(6);

        m_controller = new XboxController(0);

        DataLogManager.start();

        m_controller.povUp(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new)
                .whileTrue(m_elev.run(m_elev::up));
    }

    @Override
    public void robotInit() {
    }

    @Override
    public void teleopInit() {
        m_feedstate = Feedstate.REVERSING; // back up a bit to reset the ball position
        m_state_counter = 0;
    }

    @Override
    public void teleopPeriodic() {
        // FEED
        m_advance_feed = m_controller.getLeftBumper(); // advance feed
        if (m_advance_feed) {
            m_state_counter = 0;
            switch (m_feedstate) {
                case OFF:
                    m_feedstate = Feedstate.ADVANCING;
                    break;
                case REVERSING:
                case ADVANCING:
                default:
                    // m_feedstate = Feedstate.OFF;
            }
        }
        if (m_state_counter > FEED_DURATION) { // stop after 1s
            m_state_counter = 0;
            switch (m_feedstate) {
                case OFF:
                    break;
                case REVERSING:
                    m_feedstate = Feedstate.OFF;
                    break;
                case ADVANCING:
                    m_feedstate = Feedstate.REVERSING; // reset position
                    break;
                default:
                    m_feedstate = Feedstate.OFF;
            }

        }
        switch (m_feedstate) {
            case OFF:
                m_feed.set(0.0);
                break;
            case REVERSING:
                m_feed.set(-FEED_SPEED);
                break;
            case ADVANCING:
                m_feed.set(FEED_SPEED);
                break;
            default:
                // ?
        }
        m_state_counter += 1;

        // ELEVATION
        int m_elevation_up_down = m_controller.getPOV();
        SmartDashboard.putNumber("m_elevation_up_down", m_elevation_up_down);

        if (m_elevation_up_down == 0) { // push = down
            m_current_elevation -= ELEVATION_SPEED;
        } else if (m_elevation_up_down == 180) { // pull = up
            m_current_elevation += ELEVATION_SPEED;
        }
        m_elevation.set(m_current_elevation);

        // WHEELS
        double m_desired_wheel_speed_rpm = WHEEL_MAX_RPM * m_controller.getLeftTriggerAxis();
        SmartDashboard.putNumber("m_desired_wheel_speed_rpm", m_desired_wheel_speed_rpm);

        double m_controller_output1 = m_wheel_controller1.calculate(m_wheel_encoder1.getRate(),
                m_desired_wheel_speed_rpm);
        double m_controller_output2 = m_wheel_controller2.calculate(m_wheel_encoder2.getRate(),
                m_desired_wheel_speed_rpm);
        SmartDashboard.putNumber("m_controller_output1", m_controller_output1);
        SmartDashboard.putNumber("m_controller_output2", m_controller_output2);

        m_wheel1.setRaw((int) Math.max(0, Math.min(4095,
                (WHEEL_FF * m_desired_wheel_speed_rpm + m_controller_output1))));
        m_wheel2.setRaw((int) Math.max(0, Math.min(4095,
                (WHEEL_FF * m_desired_wheel_speed_rpm + m_controller_output2))));
    }

    public void periodic() {

        // FEED
        SmartDashboard.putBoolean("m_advance_feed", m_advance_feed);
        SmartDashboard.putNumber("m_state_counter", m_state_counter);
        SmartDashboard.putString("m_feedstate", m_feedstate.name());
        SmartDashboard.putNumber("m_feed", m_feed.get());

        // ELEVATIONS
        SmartDashboard.putNumber("m_current_elevation", m_current_elevation);
        SmartDashboard.putNumber("m_elevation", m_elevation.get());

        // WHEELS
        SmartDashboard.putNumber("m_wheel1 raw", m_wheel1.getRaw());
        SmartDashboard.putNumber("m_wheel2 raw", m_wheel2.getRaw());

        SmartDashboard.putNumber("m_wheel_encoder1_rate_rpm", m_wheel_encoder1.getRate());
        SmartDashboard.putNumber("m_wheel_encoder2_rate_rpm", m_wheel_encoder2.getRate());

        SmartDashboard.putNumber("m_controller_error1", m_wheel_controller1.getPositionError());
        SmartDashboard.putNumber("m_controller_error2", m_wheel_controller2.getPositionError());

    }

}
