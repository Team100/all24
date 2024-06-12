package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSubsystem implements Subsystem {
    private static final double kTolerance = 5;

    private final Servo m_swing = new Servo(0);
    private final SlewRateLimiter m_swingLimiter = new SlewRateLimiter(30);
    private final Servo m_boom = new Servo(1);
    private final SlewRateLimiter m_boomLimiter = new SlewRateLimiter(30);
    private final Servo m_stick = new Servo(2);
    private final SlewRateLimiter m_stickLimiter = new SlewRateLimiter(30);
    private final Servo m_wrist = new Servo(3);
    private final SlewRateLimiter m_wristLimiter = new SlewRateLimiter(30);
    private final Servo m_twist = new Servo(4);;
    private final Servo m_grip = new Servo(5);
    private final SlewRateLimiter m_gripLimiter = new SlewRateLimiter(30);

    private final InterpolatingDoubleTreeMap m_boomMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_stickMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_wristMap = new InterpolatingDoubleTreeMap();

    private  double m_swingSetpoint;
    private  double m_boomSetpoint;
    private  double m_stickSetpoint;
    private double m_wristSetpoint;
    private double m_gripSetpoint;

    public ArmSubsystem() {
        m_boomMap.put(3.0, 162.0);
        m_boomMap.put(4.0, 140.0);
        m_boomMap.put(7.0, 93.1);
        m_boomMap.put(9.0, 61.5);
        m_boomMap.put(11.0, 55.5);
        m_boomMap.put(13.0, 45.1);
        m_boomMap.put(15.0, 40.2);
        m_boomMap.put(17.0, 7.6);

        m_stickMap.put(3.0, 174.0);
        m_stickMap.put(4.0, 140.0);
        m_stickMap.put(7.0, 130.0);
        m_stickMap.put(9.0, 128.7);
        m_stickMap.put(11.0, 107.8);
        m_stickMap.put(13.0, 93.3);
        m_stickMap.put(15.0, 57.5);
        m_stickMap.put(17.0, 0.0);

        m_wristMap.put(3.0, 0.0);
        m_wristMap.put(4.0, 29.3);
        m_wristMap.put(7.0, 58.2);
        m_wristMap.put(9.0, 156.0);
        m_wristMap.put(11.0, 139.6);
        m_wristMap.put(13.0, 129.8);
        m_wristMap.put(15.0, 113.4);
        m_wristMap.put(17.0, 89.5);
    }

    public void init() {
        m_swing.setAngle(m_swingLimiter.calculate(50));
        m_boom.setAngle(m_boomLimiter.calculate(70));

        m_stick.setAngle(m_stickLimiter.calculate(90));
        m_wrist.setAngle(m_wristLimiter.calculate(90));
        m_twist.setAngle((0)); // not enough controller channels, leave twist fixed
        m_grip.setAngle(m_gripLimiter.calculate(180));
    }

    public void origin() {
        m_swing.setAngle(m_swingLimiter.calculate(46));
        m_boom.setAngle(m_boomLimiter.calculate(85.75));
        m_stick.setAngle(m_stickLimiter.calculate(90));
        m_wrist.setAngle(m_wristLimiter.calculate(45));
        m_twist.setAngle(120); // not enough controller channels, leave twist fixed
        m_grip.setAngle(m_gripLimiter.calculate(0));
    }

    public void prePickup() {
        m_swing.setAngle(m_swingLimiter.calculate(46));
        m_boom.setAngle(m_boomLimiter.calculate(48));
        m_stick.setAngle(m_stickLimiter.calculate(160));
        m_wrist.setAngle(m_wristLimiter.calculate(159));
        m_twist.setAngle(120); // not enough controller channels, leave twist fixed
        m_grip.setAngle(m_gripLimiter.calculate(0));
    }

    public void pickup() {
        m_swing.setAngle(m_swingLimiter.calculate(46));
        m_boom.setAngle(m_boomLimiter.calculate(48));
        m_stick.setAngle(m_stickLimiter.calculate(131));
        m_wrist.setAngle(m_wristLimiter.calculate(159));
        m_twist.setAngle(120); // not enough controller channels, leave twist fixed
        m_grip.setAngle(m_gripLimiter.calculate(0));
    }

    void setDistance(double distance) {
        setBoom(m_boomMap.get(distance));
        setStick(m_stickMap.get(distance));
        setWrist(m_wristMap.get(distance));
    }

    void setSwing(double x) {
        m_swingSetpoint = x;
        m_swing.setAngle(m_swingLimiter.calculate(x));
    }

    void setBoom(double x) {
        m_boomSetpoint = x;
        m_boom.setAngle(m_boomLimiter.calculate(x));
    }

    void setStick(double x) {
        m_stickSetpoint = x;
        m_stick.setAngle(m_stickLimiter.calculate(x));
    }

    void setWrist(double x) {
        m_wristSetpoint = x;
        m_wrist.setAngle(m_wristLimiter.calculate(x));
    }

    void setGrip(double x) {
        m_gripSetpoint = x;
        m_grip.setAngle(m_gripLimiter.calculate(x));
    }

    void addSwing(double x) {
        setSwing(m_swing.getAngle() + x);
    }

    void addBoom(double x) {
        setBoom(m_boom.getAngle() + x);
    }

    void addStick(double x) {
        setStick(m_stick.getAngle() + x);
    }

    void addWrist(double x) {
        setWrist(m_wrist.getAngle() + x);
    }

    void addGrip(double x) {
        setGrip(m_grip.getAngle() + x);
    }

    double getSwing() {
        return m_swing.getAngle();
    }

    double getBoom() {
        return m_boom.getAngle();
    }

    double getStick() {
        return m_stick.getAngle();
    }

    double getWrist() {
        return m_wrist.getAngle();
    }

    double getGrip() {
        return m_grip.getAngle();
    }

    @Override
    public void periodic() {
        System.out.printf("%5.2f %5.2f %5.2f %5.2f %5.2f\n",
                m_swing.getAngle(),
                m_boom.getAngle(),
                m_stick.getAngle(),
                m_wrist.getAngle(),
                m_grip.getAngle());
    }

    public boolean atSetpoint() {
        return Math.abs(m_swingSetpoint - getSwing()) < kTolerance
        && Math.abs(m_boomSetpoint - getBoom()) < kTolerance
        && Math.abs(m_stickSetpoint - getStick()) < kTolerance
        && Math.abs(m_wristSetpoint - getWrist()) < kTolerance
        && Math.abs(m_gripSetpoint - getGrip()) < kTolerance;
    }

}
