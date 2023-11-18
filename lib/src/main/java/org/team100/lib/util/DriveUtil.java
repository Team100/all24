package org.team100.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveUtil {
    /**
     * @param twist    [-1,1]
     * @param maxSpeed meters per second
     * @param maxRot   radians per second
     * @return meters and rad per second as specified by speed limits
     */
    public static Twist2d scale(Twist2d twist, double maxSpeed, double maxRot) {
        return new Twist2d(
                maxSpeed * MathUtil.clamp(twist.dx, -1, 1),
                maxSpeed * MathUtil.clamp(twist.dy, -1, 1),
                maxRot * MathUtil.clamp(twist.dtheta, -1, 1));
    }

    public static ChassisSpeeds scaleChassisSpeeds(Twist2d twist, double maxSpeed, double maxRot) {
        Twist2d scaled = scale(twist, maxSpeed, maxRot);
        return new ChassisSpeeds(
                scaled.dx,
                scaled.dy,
                scaled.dtheta);
    }

    private DriveUtil() {
    }
}
