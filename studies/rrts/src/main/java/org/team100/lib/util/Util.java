package org.team100.lib.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;

public class Util {

    public static <U extends Num, V extends Num> String matStr(Matrix<U, V> mat) {
        String result = "";
        for (double d : mat.getData()) {
            result += String.format("%5.3f ", d);
        }
        return result;
    }
    
}
