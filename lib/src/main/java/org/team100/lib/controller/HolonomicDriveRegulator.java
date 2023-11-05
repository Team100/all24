package org.team100.lib.controller;

import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.system.examples.DoubleIntegratorCartesian1D;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Three one-dimensional estimator/controllers for holonomic drive.
 * 
 * At the moment these are each a double integrator, which is definitely wrong.
 * 
 * TODO: use a more realistic system model.
 */
public class HolonomicDriveRegulator {
    private static final double kDt = 0.02;
    // for determining completion, this seems too large.
    // TODO: figure out what this is in practice.
    private static final double kEpsilon = 0.1;

    private final SubRegulator1D xRegulator;
    private final SubRegulator1D yRegulator;
    private final SubRegulator1D thetaRegulator;

    // states are random vectors, package private for testing.
    RandomVector<N2> xhat_x;
    RandomVector<N2> xhat_y;
    RandomVector<N2> xhat_theta;

    // references are normal vectors
    Vector<N2> r_x;
    Vector<N2> r_y;
    Vector<N2> r_theta;

    public HolonomicDriveRegulator() {
        Vector<N2> stateTolerance_x = VecBuilder.fill(0.02, 0.02);
        Vector<N1> controlTolerance_x = VecBuilder.fill(.001);
        WhiteNoiseVector<N2> wx = WhiteNoiseVector.noise2(0, 0);
        MeasurementUncertainty<N2> vx = MeasurementUncertainty.for2(0.01, 0.1);
        DoubleIntegratorCartesian1D system_x = new DoubleIntegratorCartesian1D(wx, vx);
        xRegulator = new SubRegulator1D(system_x, stateTolerance_x, controlTolerance_x);

        Vector<N2> stateTolerance_y = VecBuilder.fill(0.02, 0.02);
        Vector<N1> controlTolerance_y = VecBuilder.fill(.001);
        WhiteNoiseVector<N2> wy = WhiteNoiseVector.noise2(0, 0);
        MeasurementUncertainty<N2> vy = MeasurementUncertainty.for2(0.01, 0.1);
        DoubleIntegratorCartesian1D system_y = new DoubleIntegratorCartesian1D(wy, vy);
        yRegulator = new SubRegulator1D(system_y, stateTolerance_y, controlTolerance_y);

        Vector<N2> stateTolerance_theta = VecBuilder.fill(Math.PI / 120, Math.PI / 120);
        Vector<N1> controlTolerance_theta = VecBuilder.fill(.001);
        WhiteNoiseVector<N2> wtheta = WhiteNoiseVector.noise2(0, 0);
        MeasurementUncertainty<N2> vtheta = MeasurementUncertainty.for2(0.1, 0.1);
        DoubleIntegratorRotary1D system_theta = new DoubleIntegratorRotary1D(wtheta, vtheta);
        thetaRegulator = new SubRegulator1D(system_theta, stateTolerance_theta,
                controlTolerance_theta);

        Variance<N2> px = Variance.from2StdDev(.01, .01);
        Variance<N2> py = Variance.from2StdDev(.01, .01);
        Variance<N2> ptheta = Variance.from2StdDev(.316228, .316228);

        xhat_x = new RandomVector<>(VecBuilder.fill(0, 0), px);
        xhat_y = new RandomVector<>(VecBuilder.fill(0, 0), py);
        xhat_theta = new AngularRandomVector<>(VecBuilder.fill(0, 0), ptheta);
    }

    public boolean atReference() {
        return xhat_x.x.minus(r_x).normF() < kEpsilon
                && xhat_y.x.minus(r_y).normF() < kEpsilon
                && xhat_theta.x.minus(r_theta).normF() < kEpsilon;
    }

    /**
     * TODO make currentPose a swerve state as well, to allow velocity correction.
     * 
     * @param currentPose  robot's current pose in field coordinates
     * @param desiredState reference state
     * @return field-relative twist, meters and radians per second
     */
    public Twist2d calculate(Pose2d currentPose, SwerveState desiredState) {
        SwerveState newDesiredState = this.optimize(desiredState, currentPose.getRotation());

        xhat_x = xRegulator.correctPosition(xhat_x, currentPose.getX());
        xhat_y = yRegulator.correctPosition(xhat_y, currentPose.getY());
        xhat_theta = thetaRegulator.correctPosition(xhat_theta, currentPose.getRotation().getRadians());

        // TODO: also correct velocity.

        r_x = SubRegulator1D.getR(newDesiredState.x());
        r_y = SubRegulator1D.getR(newDesiredState.y());
        r_theta = SubRegulator1D.getR(newDesiredState.theta());

        Vector<N2> rDot_x = SubRegulator1D.getRDot(newDesiredState.x());
        Vector<N2> rDot_y = SubRegulator1D.getRDot(newDesiredState.y());
        Vector<N2> rDot_theta = SubRegulator1D.getRDot(newDesiredState.theta());

        Matrix<N1, N1> totalU_x = xRegulator.calculateTotalU(xhat_x, r_x, rDot_x, kDt);
        Matrix<N1, N1> totalU_y = xRegulator.calculateTotalU(xhat_y, r_y, rDot_y, kDt);
        Matrix<N1, N1> totalU_theta = xRegulator.calculateTotalU(xhat_theta, r_theta, rDot_theta, kDt);

        xhat_x = xRegulator.predictState(xhat_x, totalU_x, kDt);
        xhat_y = yRegulator.predictState(xhat_y, totalU_y, kDt);
        xhat_theta = thetaRegulator.predictState(xhat_theta, totalU_theta, kDt);

        return new Twist2d(totalU_x.get(0, 0), totalU_y.get(0, 0), totalU_theta.get(0, 0));
    }

    public SwerveState optimize(SwerveState desiredState, Rotation2d currentAngle) {
        double delta = desiredState.theta().x() - currentAngle.getRadians();
        if (Math.abs(delta) > Math.PI / 2) {
            return new SwerveState(
                    desiredState.x(),
                    desiredState.y(),
                    new State100(desiredState.theta().x() + Math.PI, desiredState.theta().v(),
                            desiredState.theta().a()));
        } else {
            return desiredState;
        }
    }

}
