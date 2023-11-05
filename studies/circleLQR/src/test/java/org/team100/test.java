package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class test {
     private  double m_maxVolts;
    private  double m_seconds;
    private LinearSystem<N2, N1, N1> m_plant;
    private KalmanFilter<N2, N1, N1> m_observer;
    private LinearQuadraticRegulator<N2, N1, N1> m_controller;
    public LinearSystemLoop<N2, N1, N1> m_loop;
    public TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile m_trapezoidProfile;
    public TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

    // Matrix<N2, N2> matrixA = new Matrix<>(Nat.N2(), Nat.N2());

    // Matrix<N2, N1> matrixB = new Matrix<>(Nat.N2(), Nat.N1());

    // public LinearPlantInversionFeedforward<N2, N1, N2> feedforward;
    @Test
    public void testSimple() { 
        m_seconds = .02;
        m_maxVolts = 12;
        m_constraints = new TrapezoidProfile.Constraints(
                3,
                3);
        m_plant = LinearSystemId
        .createSingleJointedArmSystem(
            DCMotor.getNEO(1), .001, 10);
        m_observer = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                m_plant,
                VecBuilder.fill(.015, .17), // How accurate we
                // think our model is, in radians and radians/sec
                VecBuilder.fill(2*Math.PI/(42*45)), // How accurate we think our encoder position
                // data is. In this case we very highly trust our encoder position reading.
                m_seconds);
        m_controller = new LinearQuadraticRegulator<>(
                m_plant,
                VecBuilder.fill( Units.degreesToRadians(0.01),  Units.degreesToRadians(1)), // qelms.
                VecBuilder.fill(12), // relms. Control effort (voltage) tolerance. Decrease this to more
                m_seconds); // Nominal time between loops. 0.020 for TimedRobot, but can be
        m_loop = new LinearSystemLoop<>(m_plant, m_controller, m_observer, m_maxVolts, m_seconds);
        m_trapezoidProfile = new TrapezoidProfile(m_constraints); 
        TrapezoidProfile.State trapezoidGoal;
        trapezoidGoal = new TrapezoidProfile.State(0, 0);
        m_lastProfiledReference = m_trapezoidProfile.calculate(m_seconds, trapezoidGoal, m_lastProfiledReference);
        m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);
        m_loop.correct(VecBuilder.fill(0));
        m_loop.predict(0.020);
        double nextVoltage = m_loop.getU(0);
    // LQRManager LQRController = new LQRManager(8*Math.PI,4*Math.PI, .015,.17, 2*Math.PI/(42*45), Units.degreesToRadians(0.01),Units.degreesToRadians(1),12,12,.02);
        assertEquals(0,nextVoltage);
    }
}
