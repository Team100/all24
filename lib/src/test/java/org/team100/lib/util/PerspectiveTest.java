package org.team100.lib.util;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;

import edu.wpi.first.cscore.CameraServerCvJNI;

public class PerspectiveTest {

    public PerspectiveTest() throws IOException {
        CameraServerCvJNI.forceLoad();
    }

    @Test
    void testPerspective() {
        // src is camera pixels (upper left is zero, x to the right, y down)
        List<Point> srcpts = new ArrayList<Point>();
        srcpts.add(new Point(-0.5, -0.5));
        srcpts.add(new Point(0.5, -0.5));
        srcpts.add(new Point(-1, 1));
        srcpts.add(new Point(1, 1));
        // dst is the floor relative to the robot (robot is zero, x ahead, y to the left)
        List<Point> dstpts = new ArrayList<Point>();
        dstpts.add(new Point(1, 1));
        dstpts.add(new Point(1, -1));
        dstpts.add(new Point(2, 1));
        dstpts.add(new Point(2, -1));
        Mat src = Converters.vector_Point2f_to_Mat(srcpts);
        Mat dst = Converters.vector_Point2f_to_Mat(dstpts);
        Mat transform = Imgproc.getPerspectiveTransform(src, dst);
        System.out.println(transform.dump());
        Mat src1 = new Mat(3,1,CvType.CV_64F);
        src1.put(0,0,0);
        src1.put(1,0,0);
        src1.put(2,0,1); // homogeneous coordinates
        Mat dst1 = transform.matMul(src1);
        System.out.println(dst1.dump());
    }

}
