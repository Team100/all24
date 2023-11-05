package org.team100.frc2023;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// TODO: what is this?
public class LQRManager {
    private LinearSystem<N2, N1, N1> m_plant;
    private KalmanFilter<N2, N1, N1> m_observer;
    private LinearQuadraticRegulator<N2, N1, N1> m_controller;
    public LinearSystemLoop<N2, N1, N1> m_loop;
    public final TrapezoidProfile.Constraints m_constraints;

    Matrix<N2, N2> matrixA = new Matrix<>(Nat.N2(), Nat.N2());

    Matrix<N2, N1> matrixB = new Matrix<>(Nat.N2(), Nat.N1());

    public LinearPlantInversionFeedforward<N2,N1,N2> feedforward;

    
    public LQRManager(LinearSystem<N2, N1, N1> plant,  KalmanFilter<N2, N1, N1> observer, LinearQuadraticRegulator<N2, N1, N1> controller, TrapezoidProfile.Constraints constraints){
        m_plant = plant;
        m_observer = observer;
        m_controller = controller;
        m_constraints = constraints;
        m_loop = new LinearSystemLoop<>(m_plant, m_controller, m_observer, 12.0, 0.020);
    }

    public void setAMatrix(double tr, double br){
        matrixA.set(0, 1, tr);
        matrixA.set(1, 1, br);
    }

    public void setBMatrix(double t, double b){
        matrixB.set(0, 0, t);
        matrixB.set(1, 0, b);

    }

    public void createFeedforward(){
        feedforward = new LinearPlantInversionFeedforward<>(matrixA, matrixB, 0.020);
    }
}