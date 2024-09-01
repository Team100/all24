# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913
# really to test numeric differentation
# see testPose3.cpp

import math

import unittest
import numpy as np
from gtsam import Point3, Pose3, Rot3
from numpy.testing import assert_almost_equal


P = Point3(0.2,0.7,-2)
R = Rot3.Rodrigues(0.3,0,0)
P2 = Point3(3.5,-8.2,4.2)
T = Pose3(R,P2)
T2 = Pose3(Rot3.Rodrigues(0.3,0.2,0.1),P2)
T3 = Pose3(Rot3.Rodrigues(-90, 0, 0), Point3(1, 2, 3))
tol=1e-5

# some shared test values - pulled from equivalent test in Pose2
l1 = Point3(1, 0, 0)
l2 = Point3(1, 1, 0)
l3 = Point3(2, 2, 0)
l4 = Point3(1, 4,-4)
x1 = Pose3()
x2 = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), l2)
x3 = Pose3(Rot3.Ypr(math.pi/4.0, 0.0, 0.0), l2)

xl1 = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(1, 0, 0))
xl2 = Pose3(Rot3.Ypr(0.0, 1.0, 0.0), Point3(1, 1, 0))
xl3 = Pose3(Rot3.Ypr(1.0, 0.0, 0.0), Point3(2, 2, 0))
xl4 = Pose3(Rot3.Ypr(0.0, 0.0, 1.0), Point3(1, 4,-4))

def transformFrom_( pose: Pose3,   point: Point3)->Point3:
    return pose.transformFrom(point)


class TestPose3(unittest.TestCase):














    # Check Adjoint numerical derivatives
    def test_Adjoint_jacobians(self) -> None:
        xi = np.array([ 0.1, 1.2, 2.3, 3.1, 1.4, 4.5])

        # Check evaluation sanity check
        assert_almost_equal((T.AdjointMap().compose(xi)), T.Adjoint(xi))
        assert_almost_equal((T2.AdjointMap().compose(xi)), T2.Adjoint(xi))
        assert_almost_equal((T3.AdjointMap().compose(xi)), T3.Adjoint(xi))

        # Check jacobians
        actualH1 = np.zeros((1, 1), order="F")
        actualH2 = np.zeros((1, 1), order="F")
        def Adjoint_proxy(T: Pose3, xi: np.array) -> np.array:
            return T.Adjoint(xi)
        T.Adjoint(xi, actualH1, actualH2);
        expectedH1 = numericalDerivative21(Adjoint_proxy, T, xi);
        expectedH2 = numericalDerivative22(Adjoint_proxy, T, xi);
        assert_almost_equal(expectedH1, actualH1)
        assert_almost_equal(expectedH2, actualH2)

        T2.Adjoint(xi, actualH1, actualH2);
        expectedH1 = numericalDerivative21(Adjoint_proxy, T2, xi);
        expectedH2 = numericalDerivative22(Adjoint_proxy, T2, xi);
        assert_almost_equal(expectedH1, actualH1)
        assert_almost_equal(expectedH2, actualH2)

        T3.Adjoint(xi, actualH1, actualH2);
        expectedH1 = numericalDerivative21(Adjoint_proxy, T3, xi);
        expectedH2 = numericalDerivative22(Adjoint_proxy, T3, xi);
        assert_almost_equal(expectedH1, actualH1)
        assert_almost_equal(expectedH2, actualH2)






    # Check AdjointTranspose and jacobians
    def test_AdjointTranspose(self) -> None:
        xi =  np.array([0.1, 1.2, 2.3, 3.1, 1.4, 4.5])

        # Check evaluation
        assert_almost_equal((T.AdjointMap().transpose().compose(xi)),
                T.AdjointTranspose(xi))
        assert_almost_equal((T2.AdjointMap().transpose().compose(xi)),
                T2.AdjointTranspose(xi))
        assert_almost_equal((T3.AdjointMap().transpose().compose(xi)),
                T3.AdjointTranspose(xi))

        # Check jacobians
        actualH1 = np.zeros((1, 1), order="F")
        actualH2 = np.zeros((1, 1), order="F")
        def AdjointTranspose_proxy(T: np.array,  xi: np.array) -> np.array:
            return T.AdjointTranspose(xi)
            

        T.AdjointTranspose(xi, actualH1, actualH2);
        expectedH1 = numericalDerivative21(AdjointTranspose_proxy, T, xi);
        expectedH2 = numericalDerivative22(AdjointTranspose_proxy, T, xi);
        assert_almost_equal(expectedH1, actualH1, 8)
        assert_almost_equal(expectedH2, actualH2)

        T2.AdjointTranspose(xi, actualH1, actualH2);
        expectedH1 = numericalDerivative21(AdjointTranspose_proxy, T2, xi);
        expectedH2 = numericalDerivative22(AdjointTranspose_proxy, T2, xi);
        assert_almost_equal(expectedH1, actualH1, 8)
        assert_almost_equal(expectedH2, actualH2)

        T3.AdjointTranspose(xi, actualH1, actualH2);
        expectedH1 = numericalDerivative21(AdjointTranspose_proxy, T3, xi);
        expectedH2 = numericalDerivative22(AdjointTranspose_proxy, T3, xi);
        assert_almost_equal(expectedH1, actualH1, 8)
        assert_almost_equal(expectedH2, actualH2)




    # Check translation and its pushforward
    def test_translation(self) -> None:
        actualH = np.zeros((1, 1), order="F")
        assert_almost_equal(Point3(3.5, -8.2, 4.2), T.translation(actualH), 8)

        def f(T: Pose3) -> Point3:
            return T.translation()
        numericalH = numericalDerivative11<Point3, Pose3>(f, T);
        assert_almost_equal(numericalH, actualH, 6)




    # Check rotation and its pushforward
    def test_rotation(self) -> None:
        actualH = np.zeros((1, 1), order="F")
        assert_almost_equal(R, T.rotation(actualH), 8)

        def f(T: Pose3) -> Pose3:
            return T.rotation()
        numericalH = numericalDerivative11<Rot3, Pose3>(f, T);
        assert_almost_equal(numericalH, actualH, 6)





    # Check compose and its pushforward
    def test_compose(self) -> None:
        actual: np.array = T2.compose(T2).matrix() # 4x4
        expected: np.array = T2.matrix().dot(T2.matrix()) # 4x4
        assert_almost_equal(actual,expected,8)

        actualDcompose1 = np.zeros((1, 1), order="F")
        actualDcompose2 = np.zeros((1, 1), order="F")
        T2.compose(T2, actualDcompose1, actualDcompose2)

        numericalH1 = numericalDerivative21(Pose3.compose, T2, T2);
        assert_almost_equal(numericalH1,actualDcompose1,3)
        assert_almost_equal(T2.inverse().AdjointMap(),actualDcompose1,3)

        numericalH2 = numericalDerivative22(Pose3.compose, T2, T2);
        assert_almost_equal(numericalH2,actualDcompose2,4)






    # Check compose and its pushforward, another case
    def test_compose2(self) -> None:
        T1: Pose3 = T
        actual: np.array = T1.compose(T2).matrix()
        expected: np.array = T1.matrix().dot(T2.matrix())
        assert_almost_equal(actual,expected,1e-8)

        actualDcompose1 = np.zeros((1, 1), order="F")
        actualDcompose2 = np.zeros((1, 1), order="F")
        T1.compose(T2, actualDcompose1, actualDcompose2);

        numericalH1 = numericalDerivative21(Pose3.compose, T1, T2);
        assert_almost_equal(numericalH1,actualDcompose1,3)
        assert_almost_equal(T2.inverse().AdjointMap(),actualDcompose1,3)

        numericalH2 = numericalDerivative22(Pose3.compose, T1, T2);
        assert_almost_equal(numericalH2,actualDcompose2,5)






    def test_inverse(self) -> None:
        actualDinverse = np.zeros((1, 1), order="F")
        actual: np.array = T.inverse(actualDinverse).matrix();
        expected: np.array = T.matrix().inverse();
        assert_almost_equal(actual,expected,8)

        numericalH: np.array = numericalDerivative11(Pose3.inverse, T);
        assert_almost_equal(numericalH,actualDinverse,3)
        assert_almost_equal(-T.AdjointMap(),actualDinverse,3)





    def test_inverseDerivatives2(self) -> None:
        R: Rot3  = Rot3.Rodrigues(0.3,0.4,-0.5)
        t = Point3(3.5,-8.2,4.2)
        T = Pose3(R,t)

        numericalH = numericalDerivative11(Pose3.inverse, T);
        actualDinverse = np.zeros((1, 1), order="F")
        T.inverse(actualDinverse);
        assert_almost_equal(numericalH,actualDinverse,3)
        assert_almost_equal(-T.AdjointMap(),actualDinverse,3)







    def test_Dtransform_from1_a(self) -> None:
        actualDtransform_from1 = np.zeros((1, 1), order="F")
        T.transformFrom(P, actualDtransform_from1, {});
        numerical: np.array = numericalDerivative21(transformFrom_, T, P);
        assert_almost_equal(numerical, actualDtransform_from1, 8)


    def test_Dtransform_from1_b(self) -> None:
        origin = Pose3()
        actualDtransform_from1 = np.zeros((1, 1), order="F")
        origin.transformFrom(P, actualDtransform_from1, {})
        numerical = numericalDerivative21(transformFrom_, origin, P)
        assert_almost_equal(numerical, actualDtransform_from1, 8)


    def test_Dtransform_from1_c(self) -> None:
        origin = Point3(0, 0, 0)
        T0 = Pose3(R, origin)
        actualDtransform_from1 = np.zeros((1, 1), order="F")
        T0.transformFrom(P, actualDtransform_from1, {});
        numerical: np.array = numericalDerivative21(transformFrom_, T0, P);
        assert_almost_equal(numerical, actualDtransform_from1, 8)


    def test_Dtransform_from1_d(self) -> None:
        I = Rot3.Identity()
        t0 = Point3(100, 0, 0)
        T0 = Pose3(I, t0)
        actualDtransform_from1 = np.zeros((1,1), order='F')
        T0.transformFrom(P, actualDtransform_from1, {})
        numerical = numericalDerivative21(transformFrom_, T0, P)
        assert_almost_equal(numerical, actualDtransform_from1, 8)


    def test_Dtransform_from2(self) -> None:
        actualDtransform_from2 = np.zeros((1,1), order='F')
        T.transformFrom(P, {}, actualDtransform_from2)
        numerical = numericalDerivative22(transformFrom_, T, P)
        assert_almost_equal(numerical, actualDtransform_from2, 8)



    def test_Dtransform_to1(self) -> None:
        computed = np.zeros((1,1), order='F')
        T.transformTo(P, computed, {})
        numerical = numericalDerivative21(transform_to_, T, P)
        assert_almost_equal(numerical, computed, 8)


    def test_Dtransform_to2(self) -> None:
        computed = np.zeros((1,1), order='F')
        T.transformTo(P, {}, computed);
        numerical = numericalDerivative22(transform_to_, T, P);
        assert_almost_equal(numerical, computed, 8)


    def test_transform_to_with_derivatives(self) -> None:
        actH1 = np.zeros((1,1), order='F')
        actH2 = np.zeros((1,1), order='F')
        T.transformTo(P, actH1, actH2)
        expH1 = numericalDerivative21(transform_to_, T, P)
        expH2 = numericalDerivative22(transform_to_, T, P)
        assert_almost_equal(expH1, actH1, 8)
        assert_almost_equal(expH2, actH2, 8)


    def test_transform_from_with_derivatives(self) -> None:
        actH1 = np.zeros((1,1), order='F')
        actH2 = np.zeros((1,1), order='F')
        T.transformFrom(P, actH1, actH2)
        expH1 = numericalDerivative21(transformFrom_, T, P)
        expH2 = numericalDerivative22(transformFrom_, T, P)
        assert_almost_equal(expH1, actH1, 8)
        assert_almost_equal(expH2, actH2, 8)






    def test_transformPoseFrom(self) -> None:
        actual: np.array = T2.compose(T2).matrix();
        expected:np.array = T2.matrix().dot(T2.matrix())
        assert_almost_equal(actual, expected, 8)

        H1 = np.zeros((1,1), order='F')
        H2 = np.zeros((1,1), order='F')
        T2.transformPoseFrom(T2, H1, H2)

        numericalH1: np.array = numericalDerivative21(transformPoseFrom_, T2, T2)
        assert_almost_equal(numericalH1, H1, 3)
        assert_almost_equal(T2.inverse().AdjointMap(), H1, 3)

        numericalH2: np.array = numericalDerivative22(transformPoseFrom_, T2, T2)
        assert_almost_equal(numericalH2, H2, 4)






    def test_transformPoseTo_with_derivatives(self) -> None:
        actH1 = np.zeros((1, 1), order="F")
        actH2 = np.zeros((1, 1), order="F")
        res: Pose3 = T.transformPoseTo(T2, actH1, actH2)
        assert_almost_equal(res, T.inverse().compose(T2))

        expH1 = numericalDerivative21(transformPoseTo_, T, T2)
        expH2 = numericalDerivative22(transformPoseTo_, T, T2)
        assert_almost_equal(expH1, actH1, 8)
        assert_almost_equal(expH2, actH2, 8)


    def test_transformPoseTo_with_derivatives2(self) -> None:
        actH1 = np.zeros((1, 1), order="F")
        actH2 = np.zeros((1, 1), order="F")
        res: Pose3 = T.transformPoseTo(T3, actH1, actH2)
        assert_almost_equal(res, T.inverse().compose(T3))

        expH1 = numericalDerivative21(transformPoseTo_, T, T3)
        expH2 = numericalDerivative22(transformPoseTo_, T, T3)
        assert_almost_equal(expH1, actH1, 8)
        assert_almost_equal(expH2, actH2, 8)







    def test_between(self) -> None:
        expected: Pose3 = T2.inverse() * T3;
        actualDBetween1 = np.zeros((1,1), order='F')
        actualDBetween2 = np.zeros((1,1), order='F')
        actual: Pose3 = T2.between(T3, actualDBetween1,actualDBetween2);
        assert_almost_equal(expected,actual)

        numericalH1: np.array = numericalDerivative21(Pose3.between , T2, T3)
        assert_almost_equal(numericalH1,actualDBetween1,3)

        numericalH2: np.array = numericalDerivative22(Pose3.between , T2, T3)
        assert_almost_equal(numericalH2,actualDBetween2,5)






    def test_range(self) -> None:
        actualH1 = np.zeros((1,1), order='F')
        actualH2 = np.zeros((1,1), order='F')

        # establish range is indeed zero
        self.assertAlmostEqual(1,x1.range(l1),9)

        # establish range is indeed sqrt2
        self.assertAlmostEqual(math.sqrt(2.0),x1.range(l2),9)

        # Another pair
        actual23: float = x2.range(l3, actualH1, actualH2);
        self.assertAlmostEqual(math.sqrt(2.0),actual23,9)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(range_proxy, x2, l3);
        expectedH2 = numericalDerivative22(range_proxy, x2, l3);
        assert_almost_equal(expectedH1,actualH1)
        assert_almost_equal(expectedH2,actualH2)

        # Another test
        actual34: float = x3.range(l4, actualH1, actualH2);
        self.assertAlmostEqual(5,actual34,9)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(range_proxy, x3, l4);
        expectedH2 = numericalDerivative22(range_proxy, x3, l4);
        assert_almost_equal(expectedH1,actualH1)
        assert_almost_equal(expectedH2,actualH2)





    def test_range_pose(self) -> None:
        actualH1 = np.zeros((1,1), order='F')
        actualH2 = np.zeros((1,1), order='F')

        # establish range is indeed zero
        self.assertAlmostEqual(1,x1.range(xl1),9)

        # establish range is indeed sqrt2
        self.assertAlmostEqual(math.sqrt(2.0),x1.range(xl2),9)

        # Another pair
        actual23: float = x2.range(xl3, actualH1, actualH2);
        self.assertAlmostEqual(math.sqrt(2.0),actual23,9)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(range_pose_proxy, x2, xl3);
        expectedH2 = numericalDerivative22(range_pose_proxy, x2, xl3);
        assert_almost_equal(expectedH1,actualH1)
        assert_almost_equal(expectedH2,actualH2)

        # Another test
        actual34: float = x3.range(xl4, actualH1, actualH2);
        self.assertAlmostEqual(5,actual34,9)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(range_pose_proxy, x3, xl4);
        expectedH2 = numericalDerivative22(range_pose_proxy, x3, xl4);
        assert_almost_equal(expectedH1,actualH1)
        assert_almost_equal(expectedH2,actualH2)





    def test_Bearing(self) -> None:
        actualH1 = np.zeros((1, 1), order="F")
        actualH2 = np.zeros((1, 1), order="F")
        assert_almost_equal(Unit3(1, 0, 0), x1.bearing(l1, actualH1, actualH2), 1e-9));

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(bearing_proxy, x1, l1);
        expectedH2 = numericalDerivative22(bearing_proxy, x1, l1);
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)


    def test_Bearing2(self) -> None:
        actualH1 = np.zeros((1, 1), order="F")
        actualH2 = np.zeros((1, 1), order="F")
        assert_almost_equal(Unit3(0,0.6,-0.8), x2.bearing(l4, actualH1, actualH2), 1e-9));

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(bearing_proxy, x2, l4);
        expectedH2 = numericalDerivative22(bearing_proxy, x2, l4);
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)


    def test_PoseToPoseBearing(self) -> None:
        actualH1 = np.zeros((1, 1), order="F")
        actualH2 = np.zeros((1, 1), order="F")
        assert_almost_equal(Unit3(0,1,0), xl1.bearing(xl2, actualH1, actualH2), 1e-9));

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(bearing_proxy, xl1, l2);

        # Since the second pose is treated as a point, the value calculated by
        # numericalDerivative22 only depends on the position of the pose. Here, we
        # calculate the Jacobian w.r.t. the second pose's position, and then augment
        # that with zeroes in the block that is w.r.t. the second pose's
        # orientation.
        H2block = numericalDerivative22(bearing_proxy, xl1, l2);
        expectedH2 = Matrix(2, 6);
        expectedH2.setZero();
        expectedH2.block<2, 3>(0, 3) = H2block;

        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)







    def test_ExpmapDerivative1(self) -> None:
        actualH = np.zeros((1, 1), order="F")
        w = np.array([  0.1, 0.2, 0.3, 4.0, 5.0, 6.0])
        Pose3.Expmap(w,actualH)
        expectedH = numericalDerivative21<Pose3, Vector6,
            OptionalJacobian<6, 6> >(Pose3.Expmap, w, {});
        assert_almost_equal(expectedH, actualH));


    def test_ExpmapDerivative2(self) -> None:
        # Iserles05an (Lie-group Methods) says:
        # scalar is easy: d exp(a(t)) / dt = exp(a(t)) a'(t)
        # matrix is hard: d exp(A(t)) / dt = exp(A(t)) dexp[-A(t)] A'(t)
        # where A(t): T -> se(3) is a trajectory in the tangent space of SE(3)
        # and dexp[A] is a linear map from 4*4 to 4*4 derivatives of se(3)
        # Hence, the above matrix equation is typed: 4*4 = SE(3) * linear_map(4*4)

        # In GTSAM, we don't work with the Lie-algebra elements A directly, but with 6-vectors.
        # xi is easy: d Expmap(xi(t)) / dt = ExmapDerivative[xi(t)] * xi'(t)

        # Let's verify the above formula.

        auto xi = [](double t) {
            Vector6 v;
            v << 2 * t, sin(t), 4 * t * t, 2 * t, sin(t), 4 * t * t;
            return v;
        };
        auto xi_dot = [](double t) {
            Vector6 v;
            v << 2, cos(t), 8 * t, 2, cos(t), 8 * t;
            return v;
        };

        # We define a function T
        auto T = [xi](double t) { return Pose3.Expmap(xi(t)); };

        for (double t = -2.0; t < 2.0; t += 0.3) {
            const Matrix expected = numericalDerivative11<Pose3, double>(T, t)
            const Matrix actual = Pose3.ExpmapDerivative(xi(t)) * xi_dot(t)
            assert_almost_equal(expected, actual, 7)
        }


    def test_ExpmapDerivativeQr(self) -> None:
        w = np.random.rand(6)
        w.head<3>().normalize();
        w.head<3>() = w.head<3>() * 0.9e-2;
        actualQr: np.array = Pose3::ComputeQforExpmapDerivative(w, 0.01);
        expectedH: np.array = numericalDerivative21<Pose3, Vector6,
            OptionalJacobian<6, 6> >(Pose3.Expmap, w, {});
        expectedQr: np.array = expectedH.bottomLeftCorner<3, 3>();
        assert_almost_equal(expectedQr, actualQr, 6)


    def test_LogmapDerivative(self) -> None:
        actualH = np.zeros((1, 1), order="F")
        w = np.array([ 0.1, 0.2, 0.3, 4.0, 5.0, 6.0])
        p: Pose3 = Pose3.Expmap(w)
        assert_almost_equal(w, Pose3::Logmap(p,actualH), 5)
        expectedH = numericalDerivative21<Vector6, Pose3,
            OptionalJacobian<6, 6> >(Pose3.Logmap, p, {});
        assert_almost_equal(expectedH, actualH)







    def test_adjoint(self) -> None:
        v = np.array([  1, 2, 3, 4, 5, 6])
        expected: np.array = testDerivAdjoint(screwPose3::xi, v);

        actualH1 = np.zeros((1, 1), order="F")
        actualH2 = np.zeros((1, 1), order="F")
        actual: np.array = Pose3.adjoint(screwPose3::xi, v, actualH1, actualH2);

        numericalH1 = numericalDerivative21<Vector6, Vector6, Vector6>(
            testDerivAdjoint, screwPose3::xi, v, 1e-5);
        numericalH2 = numericalDerivative22<Vector6, Vector6, Vector6>(
            testDerivAdjoint, screwPose3::xi, v, 1e-5);

        assert_almost_equal(expected,actual,5)
        assert_almost_equal(numericalH1,actualH1,5)
        assert_almost_equal(numericalH2,actualH2,5)




    def test_adjointTranspose(self) -> None:
        xi = np.array([ 0.01, 0.02, 0.03, 1.0, 2.0, 3.0])
        v = np.array([ 0.04, 0.05, 0.06, 4.0, 5.0, 6.0])
        expected: np.array = testDerivAdjointTranspose(xi, v);

        actualH1 = np.zeros((1, 1), order="F")
        actualH2 = np.zeros((1, 1), order="F")
        actual: np.array = Pose3.adjointTranspose(xi, v, actualH1, actualH2);

        numericalH1 = numericalDerivative21<Vector6, Vector6, Vector6>(
            testDerivAdjointTranspose, xi, v, 1e-5);
        numericalH2 = numericalDerivative22<Vector6, Vector6, Vector6>(
            testDerivAdjointTranspose, xi, v, 1e-5);

        assert_almost_equal(expected,actual,15)
        assert_almost_equal(numericalH1,actualH1,5)
        assert_almost_equal(numericalH2,actualH2,5)




    def test_interpolateJacobians1(self) -> None:
        X: Pose3 = Pose3.Identity();
        Y: Pose3 = Pose3(Rot3.Rz(math.pi/2), Point3(1, 0, 0))
        t: float = 0.5
        expectedPoseInterp = Pose3(Rot3.Rz(math.pi/4), Point3(0.5, -0.207107, 0)); # note: different from test above: this is full Pose3 interpolation
        actualJacobianX = np.zeros((1, 1), order="F")
        actualJacobianY = np.zeros((1, 1), order="F")
        assert_almost_equal(expectedPoseInterp, interpolate(X, Y, t, actualJacobianX, actualJacobianY), 1e-5));

        expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        assert_almost_equal(expectedJacobianX,actualJacobianX,1e-6));

        expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        assert_almost_equal(expectedJacobianY,actualJacobianY,1e-6));

    def test_interpolateJacobians2(self) -> None:
        X: Pose3  = Pose3.Identity()
        Y: Pose3= Pose3(Rot3.Identity(), Point3(1, 0, 0))
        t: float = 0.3
        expectedPoseInterp: Pose3 = Pose3(Rot3.Identity(), Point3(0.3, 0, 0))
        actualJacobianX = np.zeros((1, 1), order="F")
        actualJacobianY = np.zeros((1, 1), order="F")
        assert_almost_equal(expectedPoseInterp, interpolate(X, Y, t, actualJacobianX, actualJacobianY), 1e-5));

        Matrix expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        assert_almost_equal(expectedJacobianX,actualJacobianX,6)

        Matrix expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        assert_almost_equal(expectedJacobianY,actualJacobianY,6)

    def test_interpolateJacobians3(self) -> None:
        X: Pose3 = Pose3.Identity()
        Y:Pose3 = Pose3(Rot3.Rz(math.pi/2), Point3(0, 0, 0))
        t: float = 0.5
        expectedPoseInterp:Pose3 = Pose3 (Rot3::Rz(math.pi/4), Point3(0, 0, 0))
        actualJacobianX = np.zeros((1, 1), order="F")
        actualJacobianY = np.zeros((1, 1), order="F")
        assert_almost_equal(expectedPoseInterp, interpolate(X, Y, t, actualJacobianX, actualJacobianY), 1e-5));

        expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        assert_almost_equal(expectedJacobianX,actualJacobianX,1e-6));

        expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        assert_almost_equal(expectedJacobianY,actualJacobianY,1e-6));

    def test_interpolateJacobians4(self) -> None:
        X = Pose3(Rot3.Ypr(0.1,0.2,0.3), Point3(10, 5, -2))
        Y = Pose3(Rot3.Ypr(1.1,-2.2,-0.3), Point3(-5, 1, 1))
        t:float = 0.3
        expectedPoseInterp = Pose3(Rot3.Rz(math.pi/4), Point3(0, 0, 0))
        actualJacobianX = np.zeros((1, 1), order="F")
        actualJacobianY = np.zeros((1, 1), order="F")
        interpolate(X, Y, t, actualJacobianX, actualJacobianY);

        expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        assert_almost_equal(expectedJacobianX,actualJacobianX,6)

        expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        assert_almost_equal(expectedJacobianY,actualJacobianY,6)



    def test_Create(self) -> None:
        actualH1  = np.zeros((1, 1), order="F")
        actualH2  = np.zeros((1, 1), order="F")
        actual: Pose3 = Pose3.Create(R, P2, actualH1, actualH2);
        assert_almost_equal(T, actual)
        def create(R: Rot3,  t: Point3) -> Pose3:
            return Pose3.Create(R, t)
        
        assert_almost_equal(numericalDerivative21<Pose3,Rot3,Point3>(create, R, P2), actualH1, 9)
        assert_almost_equal(numericalDerivative22<Pose3,Rot3,Point3>(create, R, P2), actualH2, 9)




    def test_ExpmapChainRule(self) -> None:
        # Muliply with an arbitrary matrix and exponentiate
        M = np.array([
        [1, 2, 3, 4, 5, 6],
        [   7, 8, 9, 1, 2, 3],
            [4, 5, 6, 7, 8, 9],
            [1, 2, 3, 4, 5, 6],
            [7, 8, 9, 1, 2, 3],
            [4, 5, 6, 7, 8, 9],
            ])
        auto g = [&](const Vector6& omega) {
            return Pose3.Expmap(M*omega);
        };

        # Test the derivatives at zero
        expected = numericalDerivative11<Pose3, Vector6>(g, Z_6x1);
        assert_almost_equal<Matrix6>(expected, M, 5) # Pose3.ExpmapDerivative(Z_6x1) is identity

        # Test the derivatives at another value
        delta = np.array([ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        expected2 = numericalDerivative11<Pose3, Vector6>(g, delta);
        analytic = Pose3.ExpmapDerivative(M*delta) * M;
        assert_almost_equal<Matrix6>(expected2, analytic, 5) # note tolerance








if __name__ == "__main__":
    unittest.main()
