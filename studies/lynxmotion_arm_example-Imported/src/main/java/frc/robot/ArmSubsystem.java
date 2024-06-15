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
    private final SlewRateLimiter m_stickLimiter = new SlewRateLimiter(50);
    private final Servo m_wrist = new Servo(3);
    private final SlewRateLimiter m_wristLimiter = new SlewRateLimiter(80);
    private final Servo m_twist = new Servo(4);
    private final SlewRateLimiter m_twistLimiter = new SlewRateLimiter(40);
    private final Servo m_grip = new Servo(5);
    private final SlewRateLimiter m_gripLimiter = new SlewRateLimiter(30);

    private final InterpolatingDoubleTreeMap m_boomMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_stickMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_wristMap = new InterpolatingDoubleTreeMap();

    private final InterpolatingDoubleTreeMap m_downboomMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_downstickMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_downwristMap = new InterpolatingDoubleTreeMap();

    private double m_swingSetpoint;
    private double m_boomSetpoint;
    private double m_stickSetpoint;
    private double m_wristSetpoint;
    private double m_gripSetpoint;
    private double m_twistSetpoint;

    public ArmSubsystem() {
        m_boomMap.put(3.0, 177.0);
        m_boomMap.put(6.0, 135.0);
        m_boomMap.put(9.0, 109.0);
        m_boomMap.put(14.5, 54.7);

        m_stickMap.put(3.0, 135.3);
        m_stickMap.put(6.0, 105.0);
        m_stickMap.put(9.0, 76.3);
        m_stickMap.put(14.5, 0.0);

        m_wristMap.put(3.0, 0.0);
        m_wristMap.put(6.0, 0.0);
        m_wristMap.put(9.0, 0.0);
        m_wristMap.put(14.5, 0.0);

        m_downboomMap.put(3.0, 161.2);
        m_downboomMap.put(6.0, 106.0);
        m_downboomMap.put(9.0, 101.3);
        m_downboomMap.put(14.5, 32.0);

        m_downstickMap.put(3.0, 180.0);
        m_downstickMap.put(6.0, 116.1);
        m_downstickMap.put(9.0, 103.4);
        m_downstickMap.put(14.5, 0.0);

        m_downwristMap.put(3.0, 27.0);
        m_downwristMap.put(6.0, 21.1);
        m_downwristMap.put(9.0, 45.1);
        m_downwristMap.put(14.5, 0.0);
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
        m_twist.setAngle(104.5); // not enough controller channels, leave twist fixed
        m_grip.setAngle(m_gripLimiter.calculate(0));
    }

    public void pickup() {
        m_swing.setAngle(m_swingLimiter.calculate(46));
        m_boom.setAngle(m_boomLimiter.calculate(48));
        m_stick.setAngle(m_stickLimiter.calculate(131));
        m_wrist.setAngle(m_wristLimiter.calculate(159));
        m_twist.setAngle(104.5); // not enough controller channels, leave twist fixed
        m_grip.setAngle(m_gripLimiter.calculate(0));
    }

    void setPosition(double x, double y, boolean up) {
        double distance = Math.sqrt(x * x + y * y);
        // System.out.println("distance " + distance);
        setDistance(distance, up);
        double swing = Math.toDegrees(Math.atan2(y, x));
        // System.out.println("swing " + swing);
        setSwing(swing);
        setTwist(swing + 104.5);
    }

    void setDistance(double distance, boolean up) {
        if (up) {
            setBoom(m_boomMap.get(distance));
            setStick(m_stickMap.get(distance));
            setWrist(m_wristMap.get(distance));

        } else {
            setBoom(m_downboomMap.get(distance));
            setStick(m_downstickMap.get(distance));
            setWrist(m_downwristMap.get(distance));

        }
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

    void setTwist(double x) {
        m_twistSetpoint = x;
        m_twist.setAngle(m_twistLimiter.calculate(x));
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

    void addTwist(double x) {
        setTwist(m_twist.getAngle() + x);
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

    double getTwist() {
        return m_twist.getAngle();
    }

    @Override
    public void periodic() {
        System.out.printf("%5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\n",
               m_swingSetpoint - getSwing(),
                m_boomSetpoint - getBoom(), 
                m_stickSetpoint -  getStick(),
                m_wristSetpoint - getWrist(),                           
                m_gripSetpoint - getGrip(),
                m_twistSetpoint - getTwist());

    }

    public boolean atSetpoint() {
        return Math.abs(m_swingSetpoint - getSwing()) < kTolerance
                && Math.abs(m_boomSetpoint - getBoom()) < kTolerance
                && Math.abs(m_stickSetpoint - getStick()) < kTolerance
                && Math.abs(m_wristSetpoint - getWrist()) < kTolerance
                && Math.abs(m_gripSetpoint - getGrip()) < kTolerance
                && Math.abs(m_twistSetpoint - getTwist()) < kTolerance;
    }
}
