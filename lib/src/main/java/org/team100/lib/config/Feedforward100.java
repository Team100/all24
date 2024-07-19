package org.team100.lib.config;

/**
 * The motor feedforward model includes four constants.
 * 
 * TODO: this class overlaps with TorqueModel, so dedupe it somehow.
 * 
 * 
 * @param kV  Velocity: ratio between unloaded speed (rev/s) and voltage, so the
 *            units are VOLT-SEC/REV. This reflects the "Back EMF" aspect of a
 *            motor: it produces a voltage proportional to speed. The value is
 *            an intrinsic property of the motor.
 * @param kA  Acceleration: ratio between acceleration (rev/s^2) and voltage, so
 *            the units are VOLT-SEC^2/REV. This reflects torque production in
 *            the motor: torque is proportional to current, which is
 *            proportional to (net) voltage. The value will depend on the
 *            inertia of the mechanism.
 * @param kSS Static friction: voltage to get the motor moving from a stop, so
 *            the units are VOLTS. The value will depend on the "stickiness" of
 *            the mechanism.
 * @param kDS Dynamic friction: voltage to just barely keep the motor moving,
 *            the units are VOLTS. The value will depend on the "viscosity" of
 *            the mechanism.
 * 
 * @see https://en.wikipedia.org/wiki/Motor_constants
 * @see {@link edu.wpi.first.math.controller.SimpleMotorFeedforward} which uses
 *      a similar model, and also a discrete one which is more accurate.
 * @see {@link org.team100.lib.config.FeedforwardTest} which compares the
 *      models.
 */
public class Feedforward100 {
    private final double kV;
    private final double kA;
    private final double kSS;
    private final double kDS;
    private final double staticFrictionSpeedLimit;

    Feedforward100(double kV, double kA, double kSS, double kDS, double staticFrictionSpeedLimit) {
        this.kV = kV;
        this.kA = kA;
        this.kSS = kSS;
        this.kDS = kDS;
        this.staticFrictionSpeedLimit = staticFrictionSpeedLimit;
    }

    public static Feedforward100 makeNeo() {
        return new Feedforward100(0.122, 0.000, 0.100, 0.065, 0.5);
    }

    public static Feedforward100 makeArmPivot() {
        return new Feedforward100(
                0.2, // more kV than normal
                0.000, // ignore accel
                0.200, // lots of friction
                0.100, 0.5);
    }

    public static Feedforward100 zero() {
        return new Feedforward100(0, 0, 0, 0, 0);
    }

    public static Feedforward100 makeNeoVortex() {
        return new Feedforward100(0.122, 0.000, 0.100, 0.065, 0.5);
    }

    public static Feedforward100 makeWCPSwerveTurningFalcon() {
        return new Feedforward100(0.110, 0.000, 0.180, 0.010, 0.5);
    }

    public static Feedforward100 makeWCPSwerveTurningFalcon6() {
        return new Feedforward100(0.160, 0.000, 0.080, 0.100, 3.5);
    }

    public static Feedforward100 makeWCPSwerveDriveFalcon() {
        return new Feedforward100(0.110, 0.000, 0.375, 0.270, 0.5);
    }

    public static Feedforward100 makeWCPSwerveDriveFalcon6() {
        return new Feedforward100(0.130, 0.010, 0.374, 0.370, 0.1);
    }

    public static Feedforward100 makeAMSwerveDriveFalcon6() {
        return new Feedforward100(0.110, 0.000, 0.180, 0.010, 0.1);
    }

    public static Feedforward100 makeSimple() {
        return new Feedforward100(0.100, 0.100, 0.100, 0.100, 0.1);
    }

    public static Feedforward100 makeShooterFalcon6() {
        return new Feedforward100(0.110, 0.000, 0.000, 0.900, 0.1);
    }

    public double velocityFFVolts(double motorRev_S) {
        return kV * motorRev_S;
    }

    public double accelFFVolts(double motorRev_S_S) {
        return kA * motorRev_S_S;
    }

    public double frictionFFVolts(double currentMotorRev_S, double desiredMotorRev_S) {
        double direction = Math.signum(desiredMotorRev_S);
        if (currentMotorRev_S < staticFrictionSpeedLimit) {
            return kSS * direction;
        }
        return kDS * direction;
    }

    public static Feedforward100 makeTest() {
        return new Feedforward100(0.3, 0.000, 0.000, 0.100, 3.5);
    }

}
