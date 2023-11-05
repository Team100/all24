package org.team100.lib.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;

public class DriveUtil {
    /**
     * @param twist [-1,1]
     * @param maxSpeed meters per second
     * @param maxRot radians per second
     * @return meters and rad per second as specified by speed limits
     */
    public static Twist2d scale(Twist2d twist, double maxSpeed, double maxRot) {
        return new Twist2d(
                maxSpeed * MathUtil.clamp(twist.dx, -1, 1),
                maxSpeed * MathUtil.clamp(twist.dy, -1, 1),
                maxRot * MathUtil.clamp(twist.dtheta, -1, 1));
    }

    private DriveUtil() {}
}
