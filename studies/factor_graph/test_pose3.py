# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913
# really to test numeric differentation
# see testPose3.cpp

import math

import unittest
import numpy as np
from gtsam import Point2, Pose2, Rot2  # really np.array
from numpy.testing import assert_almost_equal

class TestPose3(unittest.TestCase):














# Check Adjoint numerical derivatives
def test_Adjoint_jacobians(self) -> None:
    Vector6 xi = (Vector6() << 0.1, 1.2, 2.3, 3.1, 1.4, 4.5).finished();

    # Check evaluation sanity check
    EQUALITY(static_cast<gtsam::Vector>(T.AdjointMap() * xi), T.Adjoint(xi));
    EQUALITY(static_cast<gtsam::Vector>(T2.AdjointMap() * xi), T2.Adjoint(xi));
    EQUALITY(static_cast<gtsam::Vector>(T3.AdjointMap() * xi), T3.Adjoint(xi));

    # Check jacobians
    Matrix6 actualH1, actualH2, expectedH1, expectedH2;
    std::function<Vector6(const Pose3&, const Vector6&)> Adjoint_proxy =
        [&](const Pose3& T, const Vector6& xi) { return T.Adjoint(xi); };

    T.Adjoint(xi, actualH1, actualH2);
    expectedH1 = numericalDerivative21(Adjoint_proxy, T, xi);
    expectedH2 = numericalDerivative22(Adjoint_proxy, T, xi);
    EXPECT(assert_equal(expectedH1, actualH1));
    EXPECT(assert_equal(expectedH2, actualH2));

    T2.Adjoint(xi, actualH1, actualH2);
    expectedH1 = numericalDerivative21(Adjoint_proxy, T2, xi);
    expectedH2 = numericalDerivative22(Adjoint_proxy, T2, xi);
    EXPECT(assert_equal(expectedH1, actualH1));
    EXPECT(assert_equal(expectedH2, actualH2));

    T3.Adjoint(xi, actualH1, actualH2);
    expectedH1 = numericalDerivative21(Adjoint_proxy, T3, xi);
    expectedH2 = numericalDerivative22(Adjoint_proxy, T3, xi);
    EXPECT(assert_equal(expectedH1, actualH1));
    EXPECT(assert_equal(expectedH2, actualH2));






# Check AdjointTranspose and jacobians
def test_AdjointTranspose(self) -> None:
    Vector6 xi = (Vector6() << 0.1, 1.2, 2.3, 3.1, 1.4, 4.5).finished();

    # Check evaluation
    EQUALITY(static_cast<Vector>(T.AdjointMap().transpose() * xi),
            T.AdjointTranspose(xi));
    EQUALITY(static_cast<Vector>(T2.AdjointMap().transpose() * xi),
            T2.AdjointTranspose(xi));
    EQUALITY(static_cast<Vector>(T3.AdjointMap().transpose() * xi),
            T3.AdjointTranspose(xi));

    # Check jacobians
    Matrix6 actualH1, actualH2, expectedH1, expectedH2;
    std::function<Vector6(const Pose3&, const Vector6&)> AdjointTranspose_proxy =
        [&](const Pose3& T, const Vector6& xi) {
            return T.AdjointTranspose(xi);
        };

    T.AdjointTranspose(xi, actualH1, actualH2);
    expectedH1 = numericalDerivative21(AdjointTranspose_proxy, T, xi);
    expectedH2 = numericalDerivative22(AdjointTranspose_proxy, T, xi);
    EXPECT(assert_equal(expectedH1, actualH1, 1e-8));
    EXPECT(assert_equal(expectedH2, actualH2));

    T2.AdjointTranspose(xi, actualH1, actualH2);
    expectedH1 = numericalDerivative21(AdjointTranspose_proxy, T2, xi);
    expectedH2 = numericalDerivative22(AdjointTranspose_proxy, T2, xi);
    EXPECT(assert_equal(expectedH1, actualH1, 1e-8));
    EXPECT(assert_equal(expectedH2, actualH2));

    T3.AdjointTranspose(xi, actualH1, actualH2);
    expectedH1 = numericalDerivative21(AdjointTranspose_proxy, T3, xi);
    expectedH2 = numericalDerivative22(AdjointTranspose_proxy, T3, xi);
    EXPECT(assert_equal(expectedH1, actualH1, 1e-8));
    EXPECT(assert_equal(expectedH2, actualH2));




# Check translation and its pushforward
def test_translation(self) -> None:
    Matrix actualH;
    EXPECT(assert_equal(Point3(3.5, -8.2, 4.2), T.translation(actualH), 1e-8));

    std::function<Point3(const Pose3&)> f = [](const Pose3& T) { return T.translation(); };
    Matrix numericalH = numericalDerivative11<Point3, Pose3>(f, T);
    EXPECT(assert_equal(numericalH, actualH, 1e-6));




# Check rotation and its pushforward
def test_rotation(self) -> None:
    Matrix actualH;
    EXPECT(assert_equal(R, T.rotation(actualH), 1e-8));

    std::function<Rot3(const Pose3&)> f = [](const Pose3& T) { return T.rotation(); };
    Matrix numericalH = numericalDerivative11<Rot3, Pose3>(f, T);
    EXPECT(assert_equal(numericalH, actualH, 1e-6));





# Check compose and its pushforward
# NOTE: testing::compose<Pose3>(t1,t2) = t1.compose(t2)  (see lieProxies.h)
def test_compose(self) -> None:
    Matrix actual = (T2*T2).matrix();
    Matrix expected = T2.matrix()*T2.matrix();
    EXPECT(assert_equal(actual,expected,1e-8));

    Matrix actualDcompose1, actualDcompose2;
    T2.compose(T2, actualDcompose1, actualDcompose2);

    Matrix numericalH1 = numericalDerivative21(testing::compose<Pose3>, T2, T2);
    EXPECT(assert_equal(numericalH1,actualDcompose1,5e-3));
    EXPECT(assert_equal(T2.inverse().AdjointMap(),actualDcompose1,5e-3));

    Matrix numericalH2 = numericalDerivative22(testing::compose<Pose3>, T2, T2);
    EXPECT(assert_equal(numericalH2,actualDcompose2,1e-4));






# Check compose and its pushforward, another case
def test_compose2(self) -> None:
    const Pose3& T1 = T;
    Matrix actual = (T1*T2).matrix();
    Matrix expected = T1.matrix()*T2.matrix();
    EXPECT(assert_equal(actual,expected,1e-8));

    Matrix actualDcompose1, actualDcompose2;
    T1.compose(T2, actualDcompose1, actualDcompose2);

    Matrix numericalH1 = numericalDerivative21(testing::compose<Pose3>, T1, T2);
    EXPECT(assert_equal(numericalH1,actualDcompose1,5e-3));
    EXPECT(assert_equal(T2.inverse().AdjointMap(),actualDcompose1,5e-3));

    Matrix numericalH2 = numericalDerivative22(testing::compose<Pose3>, T1, T2);
    EXPECT(assert_equal(numericalH2,actualDcompose2,1e-5));






def test_inverse(self) -> None:
  Matrix actualDinverse;
    Matrix actual = T.inverse(actualDinverse).matrix();
    Matrix expected = T.matrix().inverse();
    EXPECT(assert_equal(actual,expected,1e-8));

    Matrix numericalH = numericalDerivative11(testing::inverse<Pose3>, T);
    EXPECT(assert_equal(numericalH,actualDinverse,5e-3));
    EXPECT(assert_equal(-T.AdjointMap(),actualDinverse,5e-3));





def test_inverseDerivatives2(self) -> None:
    Rot3 R = Rot3::Rodrigues(0.3,0.4,-0.5);
    Point3 t(3.5,-8.2,4.2);
    Pose3 T(R,t);

    Matrix numericalH = numericalDerivative11(testing::inverse<Pose3>, T);
    Matrix actualDinverse;
    T.inverse(actualDinverse);
    EXPECT(assert_equal(numericalH,actualDinverse,5e-3));
    EXPECT(assert_equal(-T.AdjointMap(),actualDinverse,5e-3));







def test_Dtransform_from1_a(self) -> None:
    Matrix actualDtransform_from1;
    T.transformFrom(P, actualDtransform_from1, {});
    Matrix numerical = numericalDerivative21(transformFrom_, T, P);
    EXPECT(assert_equal(numerical, actualDtransform_from1, 1e-8));


def test_Dtransform_from1_b(self) -> None:
    Pose3 origin;
    Matrix actualDtransform_from1;
    origin.transformFrom(P, actualDtransform_from1, {});
    Matrix numerical = numericalDerivative21(transformFrom_, origin, P);
    EXPECT(assert_equal(numerical, actualDtransform_from1, 1e-8));


def test_Dtransform_from1_c(self) -> None:
    Point3 origin(0, 0, 0);
    Pose3 T0(R, origin);
    Matrix actualDtransform_from1;
    T0.transformFrom(P, actualDtransform_from1, {});
    Matrix numerical = numericalDerivative21(transformFrom_, T0, P);
    EXPECT(assert_equal(numerical, actualDtransform_from1, 1e-8));


def test_Dtransform_from1_d(self) -> None:
    Rot3 I;
    Point3 t0(100, 0, 0);
    Pose3 T0(I, t0);
    Matrix actualDtransform_from1;
    T0.transformFrom(P, actualDtransform_from1, {});
    # print(computed, "Dtransform_from1_d computed:");
    Matrix numerical = numericalDerivative21(transformFrom_, T0, P);
    # print(numerical, "Dtransform_from1_d numerical:");
    EXPECT(assert_equal(numerical, actualDtransform_from1, 1e-8));


def test_Dtransform_from2(self) -> None:
    Matrix actualDtransform_from2;
    T.transformFrom(P, {}, actualDtransform_from2);
    Matrix numerical = numericalDerivative22(transformFrom_, T, P);
    EXPECT(assert_equal(numerical, actualDtransform_from2, 1e-8));



def test_Dtransform_to1(self) -> None:
    Matrix computed;
    T.transformTo(P, computed, {});
    Matrix numerical = numericalDerivative21(transform_to_, T, P);
    EXPECT(assert_equal(numerical, computed, 1e-8));


def test_Dtransform_to2(self) -> None:
    Matrix computed;
    T.transformTo(P, {}, computed);
    Matrix numerical = numericalDerivative22(transform_to_, T, P);
    EXPECT(assert_equal(numerical, computed, 1e-8));


def test_transform_to_with_derivatives(self) -> None:
    Matrix actH1, actH2;
    T.transformTo(P, actH1, actH2);
    Matrix expH1 = numericalDerivative21(transform_to_, T, P),
            expH2 = numericalDerivative22(transform_to_, T, P);
    EXPECT(assert_equal(expH1, actH1, 1e-8));
    EXPECT(assert_equal(expH2, actH2, 1e-8));


def test_transform_from_with_derivatives(self) -> None:
    Matrix actH1, actH2;
    T.transformFrom(P, actH1, actH2);
    Matrix expH1 = numericalDerivative21(transformFrom_, T, P),
            expH2 = numericalDerivative22(transformFrom_, T, P);
    EXPECT(assert_equal(expH1, actH1, 1e-8));
    EXPECT(assert_equal(expH2, actH2, 1e-8));






def test_transformPoseFrom(self) -> None:
    Matrix actual = (T2*T2).matrix();
    Matrix expected = T2.matrix()*T2.matrix();
    EXPECT(assert_equal(actual, expected, 1e-8));

    Matrix H1, H2;
    T2.transformPoseFrom(T2, H1, H2);

    Matrix numericalH1 = numericalDerivative21(transformPoseFrom_, T2, T2);
    EXPECT(assert_equal(numericalH1, H1, 5e-3));
    EXPECT(assert_equal(T2.inverse().AdjointMap(), H1, 5e-3));

    Matrix numericalH2 = numericalDerivative22(transformPoseFrom_, T2, T2);
    EXPECT(assert_equal(numericalH2, H2, 1e-4));






def test_transformPoseTo_with_derivatives(self) -> None:
    Matrix actH1, actH2;
    Pose3 res = T.transformPoseTo(T2, actH1, actH2);
    EXPECT(assert_equal(res, T.inverse().compose(T2)));

    Matrix expH1 = numericalDerivative21(transformPoseTo_, T, T2),
            expH2 = numericalDerivative22(transformPoseTo_, T, T2);
    EXPECT(assert_equal(expH1, actH1, 1e-8));
    EXPECT(assert_equal(expH2, actH2, 1e-8));


def test_transformPoseTo_with_derivatives2(self) -> None:
    Matrix actH1, actH2;
    Pose3 res = T.transformPoseTo(T3, actH1, actH2);
    EXPECT(assert_equal(res, T.inverse().compose(T3)));

    Matrix expH1 = numericalDerivative21(transformPoseTo_, T, T3),
            expH2 = numericalDerivative22(transformPoseTo_, T, T3);
    EXPECT(assert_equal(expH1, actH1, 1e-8));
    EXPECT(assert_equal(expH2, actH2, 1e-8));







def test_between(self) -> None:
    Pose3 expected = T2.inverse() * T3;
    Matrix actualDBetween1,actualDBetween2;
    Pose3 actual = T2.between(T3, actualDBetween1,actualDBetween2);
    EXPECT(assert_equal(expected,actual));

    Matrix numericalH1 = numericalDerivative21(testing::between<Pose3> , T2, T3);
    EXPECT(assert_equal(numericalH1,actualDBetween1,5e-3));

    Matrix numericalH2 = numericalDerivative22(testing::between<Pose3> , T2, T3);
    EXPECT(assert_equal(numericalH2,actualDBetween2,1e-5));






def test_range(self) -> None:
    Matrix expectedH1, actualH1, expectedH2, actualH2;

    # establish range is indeed zero
    EXPECT_DOUBLES_EQUAL(1,x1.range(l1),1e-9);

    # establish range is indeed sqrt2
    EXPECT_DOUBLES_EQUAL(sqrt(2.0),x1.range(l2),1e-9);

    # Another pair
    double actual23 = x2.range(l3, actualH1, actualH2);
    EXPECT_DOUBLES_EQUAL(sqrt(2.0),actual23,1e-9);

    # Check numerical derivatives
    expectedH1 = numericalDerivative21(range_proxy, x2, l3);
    expectedH2 = numericalDerivative22(range_proxy, x2, l3);
    EXPECT(assert_equal(expectedH1,actualH1));
    EXPECT(assert_equal(expectedH2,actualH2));

    # Another test
    double actual34 = x3.range(l4, actualH1, actualH2);
    EXPECT_DOUBLES_EQUAL(5,actual34,1e-9);

    # Check numerical derivatives
    expectedH1 = numericalDerivative21(range_proxy, x3, l4);
    expectedH2 = numericalDerivative22(range_proxy, x3, l4);
    EXPECT(assert_equal(expectedH1,actualH1));
    EXPECT(assert_equal(expectedH2,actualH2));





def test_range_pose(self) -> None:
    Matrix expectedH1, actualH1, expectedH2, actualH2;

    # establish range is indeed zero
    EXPECT_DOUBLES_EQUAL(1,x1.range(xl1),1e-9);

    # establish range is indeed sqrt2
    EXPECT_DOUBLES_EQUAL(sqrt(2.0),x1.range(xl2),1e-9);

    # Another pair
    double actual23 = x2.range(xl3, actualH1, actualH2);
    EXPECT_DOUBLES_EQUAL(sqrt(2.0),actual23,1e-9);

    # Check numerical derivatives
    expectedH1 = numericalDerivative21(range_pose_proxy, x2, xl3);
    expectedH2 = numericalDerivative22(range_pose_proxy, x2, xl3);
    EXPECT(assert_equal(expectedH1,actualH1));
    EXPECT(assert_equal(expectedH2,actualH2));

    # Another test
    double actual34 = x3.range(xl4, actualH1, actualH2);
    EXPECT_DOUBLES_EQUAL(5,actual34,1e-9);

    # Check numerical derivatives
    expectedH1 = numericalDerivative21(range_pose_proxy, x3, xl4);
    expectedH2 = numericalDerivative22(range_pose_proxy, x3, xl4);
    EXPECT(assert_equal(expectedH1,actualH1));
    EXPECT(assert_equal(expectedH2,actualH2));





def test_Bearing(self) -> None:
    Matrix expectedH1, actualH1, expectedH2, actualH2;
    EXPECT(assert_equal(Unit3(1, 0, 0), x1.bearing(l1, actualH1, actualH2), 1e-9));

    # Check numerical derivatives
    expectedH1 = numericalDerivative21(bearing_proxy, x1, l1);
    expectedH2 = numericalDerivative22(bearing_proxy, x1, l1);
    EXPECT(assert_equal(expectedH1, actualH1, 1e-5));
    EXPECT(assert_equal(expectedH2, actualH2, 1e-5));


def test_Bearing2(self) -> None:
    Matrix expectedH1, actualH1, expectedH2, actualH2;
    EXPECT(assert_equal(Unit3(0,0.6,-0.8), x2.bearing(l4, actualH1, actualH2), 1e-9));

    # Check numerical derivatives
    expectedH1 = numericalDerivative21(bearing_proxy, x2, l4);
    expectedH2 = numericalDerivative22(bearing_proxy, x2, l4);
    EXPECT(assert_equal(expectedH1, actualH1, 1e-5));
    EXPECT(assert_equal(expectedH2, actualH2, 1e-5));


def test_PoseToPoseBearing(self) -> None:
    Matrix expectedH1, actualH1, expectedH2, actualH2, H2block;
    EXPECT(assert_equal(Unit3(0,1,0), xl1.bearing(xl2, actualH1, actualH2), 1e-9));

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

    EXPECT(assert_equal(expectedH1, actualH1, 1e-5));
    EXPECT(assert_equal(expectedH2, actualH2, 1e-5));







def test_ExpmapDerivative1(self) -> None:
    Matrix6 actualH;
    Vector6 w; w << 0.1, 0.2, 0.3, 4.0, 5.0, 6.0;
    Pose3::Expmap(w,actualH);
    Matrix expectedH = numericalDerivative21<Pose3, Vector6,
        OptionalJacobian<6, 6> >(&Pose3::Expmap, w, {});
    EXPECT(assert_equal(expectedH, actualH));


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
    auto T = [xi](double t) { return Pose3::Expmap(xi(t)); };

    for (double t = -2.0; t < 2.0; t += 0.3) {
        const Matrix expected = numericalDerivative11<Pose3, double>(T, t);
        const Matrix actual = Pose3::ExpmapDerivative(xi(t)) * xi_dot(t);
        CHECK(assert_equal(expected, actual, 1e-7));
    }


def test_ExpmapDerivativeQr(self) -> None:
    Vector6 w = Vector6::Random();
    w.head<3>().normalize();
    w.head<3>() = w.head<3>() * 0.9e-2;
    Matrix3 actualQr = Pose3::ComputeQforExpmapDerivative(w, 0.01);
    Matrix expectedH = numericalDerivative21<Pose3, Vector6,
        OptionalJacobian<6, 6> >(&Pose3::Expmap, w, {});
    Matrix3 expectedQr = expectedH.bottomLeftCorner<3, 3>();
    EXPECT(assert_equal(expectedQr, actualQr, 1e-6));


def test_LogmapDerivative(self) -> None:
    Matrix6 actualH;
    Vector6 w; w << 0.1, 0.2, 0.3, 4.0, 5.0, 6.0;
    Pose3 p = Pose3::Expmap(w);
    EXPECT(assert_equal(w, Pose3::Logmap(p,actualH), 1e-5));
    Matrix expectedH = numericalDerivative21<Vector6, Pose3,
        OptionalJacobian<6, 6> >(&Pose3::Logmap, p, {});
    EXPECT(assert_equal(expectedH, actualH));







def test_adjoint(self) -> None:
    Vector6 v = (Vector6() << 1, 2, 3, 4, 5, 6).finished();
    Vector expected = testDerivAdjoint(screwPose3::xi, v);

    Matrix actualH1, actualH2;
    Vector actual = Pose3::adjoint(screwPose3::xi, v, actualH1, actualH2);

    Matrix numericalH1 = numericalDerivative21<Vector6, Vector6, Vector6>(
        testDerivAdjoint, screwPose3::xi, v, 1e-5);
    Matrix numericalH2 = numericalDerivative22<Vector6, Vector6, Vector6>(
        testDerivAdjoint, screwPose3::xi, v, 1e-5);

    EXPECT(assert_equal(expected,actual,1e-5));
    EXPECT(assert_equal(numericalH1,actualH1,1e-5));
    EXPECT(assert_equal(numericalH2,actualH2,1e-5));




def test_adjointTranspose(self) -> None:
    Vector xi = (Vector(6) << 0.01, 0.02, 0.03, 1.0, 2.0, 3.0).finished();
    Vector v = (Vector(6) << 0.04, 0.05, 0.06, 4.0, 5.0, 6.0).finished();
    Vector expected = testDerivAdjointTranspose(xi, v);

    Matrix actualH1, actualH2;
    Vector actual = Pose3::adjointTranspose(xi, v, actualH1, actualH2);

    Matrix numericalH1 = numericalDerivative21<Vector6, Vector6, Vector6>(
        testDerivAdjointTranspose, xi, v, 1e-5);
    Matrix numericalH2 = numericalDerivative22<Vector6, Vector6, Vector6>(
        testDerivAdjointTranspose, xi, v, 1e-5);

    EXPECT(assert_equal(expected,actual,1e-15));
    EXPECT(assert_equal(numericalH1,actualH1,1e-5));
    EXPECT(assert_equal(numericalH2,actualH2,1e-5));




def test_interpolateJacobians(self) -> None:
    {
        Pose3 X = Pose3::Identity();
        Pose3 Y(Rot3::Rz(M_PI_2), Point3(1, 0, 0));
        double t = 0.5;
        Pose3 expectedPoseInterp(Rot3::Rz(M_PI_4), Point3(0.5, -0.207107, 0)); # note: different from test above: this is full Pose3 interpolation
        Matrix actualJacobianX, actualJacobianY;
        EXPECT(assert_equal(expectedPoseInterp, interpolate(X, Y, t, actualJacobianX, actualJacobianY), 1e-5));

        Matrix expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        EXPECT(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

        Matrix expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        EXPECT(assert_equal(expectedJacobianY,actualJacobianY,1e-6));
    }
    {
        Pose3 X = Pose3::Identity();
        Pose3 Y(Rot3::Identity(), Point3(1, 0, 0));
        double t = 0.3;
        Pose3 expectedPoseInterp(Rot3::Identity(), Point3(0.3, 0, 0));
        Matrix actualJacobianX, actualJacobianY;
        EXPECT(assert_equal(expectedPoseInterp, interpolate(X, Y, t, actualJacobianX, actualJacobianY), 1e-5));

        Matrix expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        EXPECT(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

        Matrix expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        EXPECT(assert_equal(expectedJacobianY,actualJacobianY,1e-6));
    }
    {
        Pose3 X = Pose3::Identity();
        Pose3 Y(Rot3::Rz(M_PI_2), Point3(0, 0, 0));
        double t = 0.5;
        Pose3 expectedPoseInterp(Rot3::Rz(M_PI_4), Point3(0, 0, 0));
        Matrix actualJacobianX, actualJacobianY;
        EXPECT(assert_equal(expectedPoseInterp, interpolate(X, Y, t, actualJacobianX, actualJacobianY), 1e-5));

        Matrix expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        EXPECT(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

        Matrix expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        EXPECT(assert_equal(expectedJacobianY,actualJacobianY,1e-6));
    }
    {
        Pose3 X(Rot3::Ypr(0.1,0.2,0.3), Point3(10, 5, -2));
        Pose3 Y(Rot3::Ypr(1.1,-2.2,-0.3), Point3(-5, 1, 1));
        double t = 0.3;
        Pose3 expectedPoseInterp(Rot3::Rz(M_PI_4), Point3(0, 0, 0));
        Matrix actualJacobianX, actualJacobianY;
        interpolate(X, Y, t, actualJacobianX, actualJacobianY);

        Matrix expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        EXPECT(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

        Matrix expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
        EXPECT(assert_equal(expectedJacobianY,actualJacobianY,1e-6));
    }


def test_Create(self) -> None:
    Matrix63 actualH1, actualH2;
    Pose3 actual = Pose3::Create(R, P2, actualH1, actualH2);
    EXPECT(assert_equal(T, actual));
    std::function<Pose3(Rot3, Point3)> create = [](Rot3 R, Point3 t) {
        return Pose3::Create(R, t);
    };
    EXPECT(assert_equal(numericalDerivative21<Pose3,Rot3,Point3>(create, R, P2), actualH1, 1e-9));
    EXPECT(assert_equal(numericalDerivative22<Pose3,Rot3,Point3>(create, R, P2), actualH2, 1e-9));




def test_ExpmapChainRule(self) -> None:
    # Muliply with an arbitrary matrix and exponentiate
    Matrix6 M;
    M << 1, 2, 3, 4, 5, 6, #
        7, 8, 9, 1, 2, 3, #
        4, 5, 6, 7, 8, 9, #
        1, 2, 3, 4, 5, 6, #
        7, 8, 9, 1, 2, 3, #
        4, 5, 6, 7, 8, 9;
    auto g = [&](const Vector6& omega) {
        return Pose3::Expmap(M*omega);
    };

    # Test the derivatives at zero
    const Matrix6 expected = numericalDerivative11<Pose3, Vector6>(g, Z_6x1);
    EXPECT(assert_equal<Matrix6>(expected, M, 1e-5)); # Pose3::ExpmapDerivative(Z_6x1) is identity

    # Test the derivatives at another value
    const Vector6 delta{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    const Matrix6 expected2 = numericalDerivative11<Pose3, Vector6>(g, delta);
    const Matrix6 analytic = Pose3::ExpmapDerivative(M*delta) * M;
    EXPECT(assert_equal<Matrix6>(expected2, analytic, 1e-5)); # note tolerance








if __name__ == "__main__":
    unittest.main()
