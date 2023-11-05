package org.team100.frc2023;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class LQRManager {
    private final double m_maxVolts;
    private final double m_seconds;
    private LinearSystem<N2, N1, N1> m_plant;
    private KalmanFilter<N2, N1, N1> m_observer;
    private LinearQuadraticRegulator<N2, N1, N1> m_controller;
    public LinearSystemLoop<N2, N1, N1> m_loop;
    public TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile m_trapezoidProfile;
    public TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

    Matrix<N2, N2> matrixA = new Matrix<>(Nat.N2(), Nat.N2());

    Matrix<N2, N1> matrixB = new Matrix<>(Nat.N2(), Nat.N1());

    public LinearPlantInversionFeedforward<N2, N1, N2> feedforward;

    // public LQRManager(double maxV, double maxA, LinearSystem<N2, N1, N1> plant, double eX, double eV, double mX, double Q1, double Q2, double R) {
    //     this(maxV, maxA, plant, eX, eV, mX, Q1, Q2, R, 12, .02);
    // }

    // public LQRManager(double maxV, double maxA, LinearSystem<N2, N1, N1> plant, double eX, double eV, double mX, double Q1, double Q2, double R, double maxVolts) {
    //     this(maxV, maxA, plant, eX, eV, mX, Q1, Q2, R, maxVolts, .02);
    // }

    public LQRManager(double maxV, double maxA, double eX, double eV, double mX, double Q1, double Q2, double R, double maxVolts, double seconds) {
        m_seconds = seconds;
        m_maxVolts = maxVolts;
        m_constraints = new TrapezoidProfile.Constraints(
                maxV,
                maxA);
        m_plant = LinearSystemId
        .createSingleJointedArmSystem(
            DCMotor.getNEO(1), 0.43, 1);
        m_observer = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                m_plant,
                VecBuilder.fill(eX, eV), // How accurate we
                // think our model is, in radians and radians/sec
                VecBuilder.fill(mX), // How accurate we think our encoder position
                // data is. In this case we very highly trust our encoder position reading.
                m_seconds);
        m_controller = new LinearQuadraticRegulator<>(
                m_plant,
                VecBuilder.fill(Q1, Q2), // qelms.
                VecBuilder.fill(R), // relms. Control effort (voltage) tolerance. Decrease this to more
                m_seconds); // Nominal time between loops. 0.020 for TimedRobot, but can be
        m_loop = new LinearSystemLoop<>(m_plant, m_controller, m_observer, m_maxVolts, m_seconds);
        m_trapezoidProfile = new TrapezoidProfile(m_constraints);
    }

    public void setAMatrix(double tr, double br) {
        matrixA.set(0, 1, tr);
        matrixA.set(1, 1, br);
    }

    public void set(double maxV, double maxA, LinearSystem<N2, N1, N1> plant, double eX, double eV, double mX, double Q1, double Q2, double R) {
        m_constraints = new TrapezoidProfile.Constraints(
                maxV,
                maxA);
        m_plant = plant;
        m_observer = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                m_plant,
                VecBuilder.fill(eX, eV), // How accurate we
                // think our model is, in radians and radians/sec
                VecBuilder.fill(mX), // How accurate we think our encoder position
                // data is. In this case we very highly trust our encoder position reading.
                m_seconds);
        m_controller = new LinearQuadraticRegulator<>(
                m_plant,
                VecBuilder.fill(Q1, Q2), // qelms.
                VecBuilder.fill(R), // relms. Control effort (voltage) tolerance. Decrease this to more
                m_seconds); // Nominal time between loops. 0.020 for TimedRobot, but can be
        m_loop = new LinearSystemLoop<>(m_plant, m_controller, m_observer, m_maxVolts, m_seconds);
        m_trapezoidProfile = new TrapezoidProfile(m_constraints);
    }

    public void setBMatrix(double t, double b) {
        matrixB.set(0, 0, t);
        matrixB.set(1, 0, b);

    }

    public void createFeedforward() {
        feedforward = new LinearPlantInversionFeedforward<>(matrixA, matrixB, m_seconds);
    }

    public double calculate(double angle, double goalPosition, double goalVelocity) {
        TrapezoidProfile.State trapezoidGoal;
        trapezoidGoal = new TrapezoidProfile.State(goalPosition, goalVelocity);
        m_lastProfiledReference = m_trapezoidProfile.calculate(m_seconds, trapezoidGoal, m_lastProfiledReference);
        m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);
        m_loop.correct(VecBuilder.fill(angle));
        m_loop.predict(0.020);
        double nextVoltage = m_loop.getU(0);
        return nextVoltage;
    }
}