
# really to test numeric differentation
# see testPose2.cpp

import math

import unittest
import numpy as np
from gtsam import Point2, Pose2, Rot2, Vector3 # really np.array
from numpy.testing import assert_allclose


class TestPose2(unittest.TestCase):


    def test_ExpmapDerivative1(self):
        actualH = [np.zeros((3,3))]
        w = Vector3(0.1, 0.27, -0.3);
        Pose2::Expmap(w,actualH);
        Matrix3 expectedH = numericalDerivative21<Pose2, Vector3,
            OptionalJacobian<3, 3> >(&Pose2::Expmap, w, {}, 1e-2);
        assert_allclose(expectedH, actualH, 1e-5));


    def test_ExpmapDerivative2(self):
        actualH = [np.zeros((3,3))]
        w0 = Vector3(0.1, 0.27, 0.0);  # alpha = 0
        Pose2::Expmap(w0,actualH);
        Matrix3 expectedH = numericalDerivative21<Pose2, Vector3,
            OptionalJacobian<3, 3> >(&Pose2::Expmap, w0, {}, 1e-2);
        assert_allclose(expectedH, actualH, 1e-5));


    def test_LogmapDerivative1(self):
        actualH = [np.zeros((3,3))]
        w = Vector3(0.1, 0.27, -0.3);
        p: Pose2 = Pose2::Expmap(w);
        assert_allclose(w, Pose2::Logmap(p,actualH), 1e-5));
        Matrix3 expectedH = numericalDerivative21<Vector3, Pose2,
            OptionalJacobian<3, 3> >(&Pose2::Logmap, p, {}, 1e-2);
        assert_allclose(expectedH, actualH, 1e-5));


    def test_LogmapDerivative2(self):
        actualH = [np.zeros((3,3))]
        w0 = Vector3(0.1, 0.27, 0.0);  # alpha = 0
        p: Pose2 = Pose2::Expmap(w0);
        assert_allclose(w0, Pose2::Logmap(p,actualH), 1e-5));
        Matrix3 expectedH = numericalDerivative21<Vector3, Pose2,
            OptionalJacobian<3, 3> >(&Pose2::Logmap, p, {}, 1e-2);
        assert_allclose(expectedH, actualH, 1e-5));





    def test_transformTo(self):
        pose = Pose2(math.pi / 2.0, Point2(1, 2));  # robot at (1,2) looking towards y
        point = Point2(-1, 4);                   # landmark at (-1,4)

        # expected
        expected = Point2(2, 2)
        Matrix expectedH1 =
            (Matrix(2, 3) << -1.0, 0.0, 2.0, 0.0, -1.0, -2.0).finished();
        Matrix expectedH2 = (Matrix(2, 2) << 0.0, 1.0, -1.0, 0.0).finished();

        # actual
        Matrix actualH1, actualH2;
        actual: Point2 = pose.transformTo(point, actualH1, actualH2);
        assert_allclose(expected, actual));

        assert_allclose(expectedH1, actualH1));
        Matrix numericalH1 = numericalDerivative21(transformTo_, pose, point);
        assert_allclose(numericalH1, actualH1));

        assert_allclose(expectedH2, actualH2));
        Matrix numericalH2 = numericalDerivative22(transformTo_, pose, point);
        assert_allclose(numericalH2, actualH2));



    def test_transformFrom(self):
        pose = Pose2(1., 0., math.pi / 2.0);
        pt = Point2(2., 1.);
        H = [np.zeros((3,3))]
        actual: Point2 = pose.transformFrom(pt, H1, H2);

        expected = Point2(0., 2.);
        assert_allclose(expected, actual));

        H1_expected = np.array([[ 0., -1., -2.], [1., 0., -1.])
        H2_expected = np.array([[ 0., -1.], [1., 0.])

        numericalH1 = numericalDerivative21(transformFrom_, pose, pt);
        assert_allclose(H1_expected, H1));
        assert_allclose(H1_expected, numericalH1));

        numericalH2 = numericalDerivative22(transformFrom_, pose, pt);
        assert_allclose(H2_expected, H2)
        assert_allclose(H2_expected, numericalH2)



    def test_compose_a(self):

        pose1 = Pose2(math.pi/4.0, Point2(math.sqrt(0.5), math.sqrt(0.5)));
        pose2 = Pose2(math.pi/2.0, Point2(0.0, 2.0));

        Matrix actualDcompose1;
        Matrix actualDcompose2;
        actual: Pose2 = pose1.compose(pose2, actualDcompose1, actualDcompose2);

        expected = Pose2(3.0*math.pi/4.0, Point2(-math.sqrt(0.5), 3.0*math.sqrt(0.5)));
        assert_allclose(expected, actual));

        expectedH1 = np.array([
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 2.0],
            [0.0, 0.0, 1.0]
        ]);
        Matrix expectedH2 = I_3x3;
        Matrix numericalH1 = numericalDerivative21<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
        Matrix numericalH2 = numericalDerivative22<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
        assert_allclose(expectedH1,actualDcompose1));
        assert_allclose(numericalH1,actualDcompose1));
        assert_allclose(expectedH2,actualDcompose2));
        assert_allclose(numericalH2,actualDcompose2));

        point = Point2(math.sqrt(0.5), 3.0*math.sqrt(0.5))
        expected_point = Point2(-1.0, -1.0)
        actual_point1: Point2= (pose1 * pose2).transformTo(point);
        actual_point2: Point2 = pose2.transformTo(pose1.transformTo(point));
        assert_allclose(expected_point, actual_point1));
        assert_allclose(expected_point, actual_point2));



    def test_compose_b(self):
        Pose2 pose1(Rot2::fromAngle(math.pi/10.0), Point2(.75, .5));
        Pose2 pose2(Rot2::fromAngle(math.pi/4.0-math.pi/10.0), Point2(0.701289620636, 1.34933052585));

        Pose2 pose_expected(Rot2::fromAngle(math.pi/4.0), Point2(1.0, 2.0));

        Pose2 pose_actual_op = pose1 * pose2;
        Matrix actualDcompose1, actualDcompose2;
        Pose2 pose_actual_fcn = pose1.compose(pose2, actualDcompose1, actualDcompose2);

        Matrix numericalH1 = numericalDerivative21<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
        Matrix numericalH2 = numericalDerivative22<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
        assert_allclose(numericalH1,actualDcompose1)
        assert_allclose(numericalH2,actualDcompose2)

        assert_allclose(pose_expected, pose_actual_op)
        assert_allclose(pose_expected, pose_actual_fcn)



    def test_compose_c(self):
        Pose2 pose1(Rot2::fromAngle(math.pi/4.0), Point2(1.0, 1.0));
        Pose2 pose2(Rot2::fromAngle(math.pi/4.0), Point2(math.sqrt(.5), math.sqrt(.5)));

        Pose2 pose_expected(Rot2::fromAngle(math.pi/2.0), Point2(1.0, 2.0));

        Pose2 pose_actual_op = pose1 * pose2;
        Matrix actualDcompose1, actualDcompose2;
        Pose2 pose_actual_fcn = pose1.compose(pose2, actualDcompose1, actualDcompose2);

        Matrix numericalH1 = numericalDerivative21<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
        Matrix numericalH2 = numericalDerivative22<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
        assert_allclose(numericalH1,actualDcompose1)
        assert_allclose(numericalH2,actualDcompose2)

        assert_allclose(pose_expected, pose_actual_op)
        assert_allclose(pose_expected, pose_actual_fcn)
        


    def test_inverse(self):
        Point2 origin(0,0), t(1,2);
        Pose2 gTl(math.pi/2.0, t); # robot at (1,2) looking towards y

        Pose2 identity, lTg = gTl.inverse();
        assert_allclose(identity,lTg.compose(gTl))
        assert_allclose(identity,gTl.compose(lTg))

        Point2 l(4,5), g(-4,6);
        assert_allclose(g,gTl*l));
        assert_allclose(l,lTg*g));

        # Check derivative
        Matrix numericalH = numericalDerivative11<Pose2,Pose2>(testing::inverse, lTg);
        Matrix actualDinverse;
        lTg.inverse(actualDinverse);
        assert_allclose(numericalH,actualDinverse)



    def test_translation(self):
        Pose2 pose(3.5, -8.2, 4.2);

        actualH = [np.zeros((3,3))]
        assert_allclose((Vector2() << 3.5, -8.2).finished(), pose.translation(actualH), 1e-8));

        std::function<Point2(const Pose2&)> f = [](const Pose2& T) { return T.translation(); };
        Matrix numericalH = numericalDerivative11<Point2, Pose2>(f, pose);
        assert_allclose(numericalH, actualH, 1e-6));


    def test_rotation(self):
        pose = Pose2(3.5, -8.2, 4.2)

        Matrix actualH(4, 3);
        assert_allclose(Rot2(4.2), pose.rotation(actualH), 1e-8));

        std::function<Rot2(const Pose2&)> f = [](const Pose2& T) { return T.rotation(); };
        Matrix numericalH = numericalDerivative11<Rot2, Pose2>(f, pose);
        assert_allclose(numericalH, actualH, 1e-6)



    def test_between(self):
        gT1 = Pose2(math.pi/2.0, Point2(1,2)); # robot at (1,2) looking towards y
        gT2 = Pose2(math.pi, Point2(-1,4));  # robot at (-1,4) looking at negative x

        Matrix actualH1,actualH2;
        expected = Pose2(math.pi/2.0, Point2(2,2));
        actual1: Pose2 = gT1.between(gT2);
        Pose2 actual2 = gT1.between(gT2,actualH1,actualH2);
        assert_allclose(expected,actual1)
        assert_allclose(expected,actual2)

        expectedH1 = np.array([
            [0.0,-1.0,-2.0],
            [1.0, 0.0,-2.0],
            [0.0, 0.0,-1.0]
        ]);
        Matrix numericalH1 = numericalDerivative21<Pose2,Pose2,Pose2>(testing::between, gT1, gT2);
        assert_allclose(expectedH1,actualH1)
        assert_allclose(numericalH1,actualH1)
        # Assert H1 = -AdjointMap(between(p2,p1)) as in doc/math.lyx
        assert_allclose(-gT2.between(gT1).AdjointMap(),actualH1)

        expectedH2 = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]);
        Matrix numericalH2 = numericalDerivative22<Pose2,Pose2,Pose2>(testing::between, gT1, gT2);
        assert_allclose(expectedH2,actualH2)
        assert_allclose(numericalH2,actualH2)




    def test_between2(self):

        p2 = Pose2(math.pi/2.0, Point2(1,2)); # robot at (1,2) looking towards y
        p1 = Pose2(math.pi, Point2(-1,4));  # robot at (-1,4) loooking at negative x

        Matrix actualH1,actualH2;
        p1.between(p2,actualH1,actualH2);
        Matrix numericalH1 = numericalDerivative21<Pose2,Pose2,Pose2>(testing::between, p1, p2);
        assert_allclose(numericalH1,actualH1)
        Matrix numericalH2 = numericalDerivative22<Pose2,Pose2,Pose2>(testing::between, p1, p2);
        assert_allclose(numericalH2,actualH2)


    # arbitrary, non perpendicular angles to be extra safe
    def test_between3(self):

        p2 = Pose2(math.pi/3.0, Point2(1,2));
        p1 = Pose2(math.pi/6.0, Point2(-1,4));

        Matrix actualH1,actualH2;
        p1.between(p2,actualH1,actualH2);
        Matrix numericalH1 = numericalDerivative21<Pose2,Pose2,Pose2>(testing::between, p1, p2);
        assert_allclose(numericalH1,actualH1)
        Matrix numericalH2 = numericalDerivative22<Pose2,Pose2,Pose2>(testing::between, p1, p2);
        assert_allclose(numericalH2,actualH2)




    def test_bearing(self):

        Matrix expectedH1, actualH1, expectedH2, actualH2;

        # establish bearing is indeed zero
        assert_allclose(Rot2(),x1.bearing(l1))

        # establish bearing is indeed 45 degrees
        assert_allclose(Rot2::fromAngle(math.pi/4.0),x1.bearing(l2)));

        # establish bearing is indeed 45 degrees even if shifted
        Rot2 actual23 = x2.bearing(l3, actualH1, actualH2);
        assert_allclose(Rot2::fromAngle(math.pi/4.0),actual23)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(bearing_proxy, x2, l3);
        assert_allclose(expectedH1,actualH1)
        expectedH2 = numericalDerivative22(bearing_proxy, x2, l3);
        assert_allclose(expectedH2,actualH2)

        # establish bearing is indeed 45 degrees even if rotated
        Rot2 actual34 = x3.bearing(l4, actualH1, actualH2);
        assert_allclose(Rot2::fromAngle(math.pi/4.0),actual34));

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(bearing_proxy, x3, l4);
        expectedH2 = numericalDerivative22(bearing_proxy, x3, l4);
        assert_allclose(expectedH1,actualH1)
        assert_allclose(expectedH2,actualH2)



    def test_bearing_pose(self):

        Pose2 xl1(1, 0, math.pi/2.0), xl2(1, 1, math.pi), xl3(2.0, 2.0,-math.pi/2.0), xl4(1, 3, 0);

        Matrix expectedH1, actualH1, expectedH2, actualH2;

        # establish bearing is indeed zero
        assert_allclose(Rot2(),x1.bearing(xl1))

        # establish bearing is indeed 45 degrees
        assert_allclose(Rot2::fromAngle(math.pi/4.0),x1.bearing(xl2))

        # establish bearing is indeed 45 degrees even if shifted
        Rot2 actual23 = x2.bearing(xl3, actualH1, actualH2);
        assert_allclose(Rot2::fromAngle(math.pi/4.0),actual23)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(bearing_pose_proxy, x2, xl3);
        expectedH2 = numericalDerivative22(bearing_pose_proxy, x2, xl3);
        assert_allclose(expectedH1,actualH1)
        assert_allclose(expectedH2,actualH2)

        # establish bearing is indeed 45 degrees even if rotated
        Rot2 actual34 = x3.bearing(xl4, actualH1, actualH2);
        assert_allclose(Rot2::fromAngle(math.pi/4.0),actual34)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(bearing_pose_proxy, x3, xl4);
        expectedH2 = numericalDerivative22(bearing_pose_proxy, x3, xl4);
        assert_allclose(expectedH1,actualH1)
        assert_allclose(expectedH2,actualH2)




    def test_range(self):

        Matrix expectedH1, actualH1, expectedH2, actualH2;

        # establish range is indeed zero
        self.assertAlmostEqual(1,x1.range(l1),1e-9);

        # establish range is indeed 45 degrees
        self.assertAlmostEqual(math.sqrt(2.0),x1.range(l2),1e-9);

        # Another pair
        actual23: float = x2.range(l3, actualH1, actualH2);
        self.assertAlmostEqual(math.sqrt(2.0),actual23,1e-9);

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(range_proxy, x2, l3);
        expectedH2 = numericalDerivative22(range_proxy, x2, l3);
        assert_allclose(expectedH1,actualH1));
        assert_allclose(expectedH2,actualH2));

        # Another test
        actual34: float = x3.range(l4, actualH1, actualH2);
        self.assertAlmostEqual(2,actual34,1e-9);

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(range_proxy, x3, l4);
        expectedH2 = numericalDerivative22(range_proxy, x3, l4);
        assert_allclose(expectedH1,actualH1));
        assert_allclose(expectedH2,actualH2));



    def test_range_pose(self):

        Pose2 xl1(1, 0, math.pi/2.0), xl2(1, 1, math.pi), xl3(2.0, 2.0,-math.pi/2.0), xl4(1, 3, 0);

        Matrix expectedH1, actualH1, expectedH2, actualH2;

        # establish range is indeed zero
        self.assertAlmostEqual(1,x1.range(xl1),1e-9);

        # establish range is indeed 45 degrees
        self.assertAlmostEqual(math.sqrt(2.0),x1.range(xl2),1e-9);

        # Another pair
        actual23: float = x2.range(xl3, actualH1, actualH2);
        self.assertAlmostEqual(math.sqrt(2.0),actual23,1e-9);

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(range_pose_proxy, x2, xl3);
        expectedH2 = numericalDerivative22(range_pose_proxy, x2, xl3);
        assert_allclose(expectedH1,actualH1));
        assert_allclose(expectedH2,actualH2));

        # Another test
        actual34: float = x3.range(xl4, actualH1, actualH2);
        self.assertAlmostEqual(2,actual34,1e-9);

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(range_pose_proxy, x3, xl4);
        expectedH2 = numericalDerivative22(range_pose_proxy, x3, xl4);
        assert_allclose(expectedH1,actualH1));
        assert_allclose(expectedH2,actualH2));



if __name__ == "__main__":
    unittest.main()
