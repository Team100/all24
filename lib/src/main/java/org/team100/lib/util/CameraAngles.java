package org.team100.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

public class CameraAngles {
    double m_downwardAngleDegrees;
    double m_horzFOVDegrees;
    double m_vertFOVDegrees;
    double m_horzResolution;
    double m_vertResolution;
    double m_cameraHeightMeters;

    public CameraAngles(
            double downwardAngleDegrees,
            double horzFOVDegrees,
            double vertFOVDegrees,
            double horzResolution,
            double vertResolution,
            double cameraHeightMeters) {
        m_downwardAngleDegrees = downwardAngleDegrees;
        m_horzFOVDegrees = horzFOVDegrees;
        m_vertFOVDegrees = vertFOVDegrees;
        m_horzResolution = horzResolution;
        m_vertResolution = vertResolution;
        m_cameraHeightMeters = cameraHeightMeters;
    }

    /**
     * Returns a robot relative translational y value of an object in a camera given the vertical
     * pixel
     */
    public double getY(double vertPixel) {
        double y = m_cameraHeightMeters * Math.tan(vertPixel * (Math.toRadians(m_vertFOVDegrees / m_vertResolution))
                + Math.toRadians(90 - m_vertFOVDegrees - m_downwardAngleDegrees));
        return y;
    }

    /**
     * Returns a robot relative translational x value of an object in a camera given the vertical
     * and horizontal pixel
     */
    public double getX(double horzPixel, double vertPixel) {
        double y = getY(vertPixel);
        double x = y
                * Math.tan(Math.toRadians(m_horzFOVDegrees) * (horzPixel - m_horzResolution / 2) / m_horzResolution);
        return x;
    }

    /**
     * Returns a robot relative translational value of an object in a camera given the vertical
     * and horizontal pixel
     */
    public Translation2d getObjectTranslation(double horzPixel, double vertPixel) {
        return new Translation2d(getX(horzPixel, vertPixel), getY(vertPixel));
    }
}