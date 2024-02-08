package org.team100.lib.util;

public class CameraAngles {
    private final double m_downwardAngleDegrees;
    private final double m_horzFOVDegrees;
    private final double m_vertFOVDegrees;
    private final double m_horzResolution;
    private final double m_vertResolution;
    private final double m_cameraHeightMeters;
    private final double m_xOffset;
    private final double m_yOffset;

    /**
     * @param downwardAngleDegrees The angle downward the camera is in
     *                             degrees, this is including half of the
     *                             view of the camera
     * @param horzFOVDegrees       The horizontal field of view of the camera
     *                             in degrees
     * @param vertFOVDegrees       The vertical field of view of the camera
     *                             in degrees
     * @param horzResolution       The amount of pixels horizontally across
     *                             the video of the PI
     * @param vertResolution       The amount of pixels vertically across the
     *                             video of the PI
     * @param cameraHeightMeters   The height of the camera in meters
     * @param xOffset              The x offset of the camera in meters relative to
     *                             the center of the drivetrain
     * @param yOffset              The y offset of the camera in meters relative to
     *                             the center of the drivetrain
     */
    public CameraAngles(
            double downwardAngleDegrees,
            double horzFOVDegrees,
            double vertFOVDegrees,
            double horzResolution,
            double vertResolution,
            double cameraHeightMeters,
            double xOffset,
            double yOffset) {
        m_downwardAngleDegrees = downwardAngleDegrees;
        m_horzFOVDegrees = horzFOVDegrees;
        m_vertFOVDegrees = vertFOVDegrees;
        m_horzResolution = horzResolution;
        m_vertResolution = vertResolution;
        m_cameraHeightMeters = cameraHeightMeters;
        m_xOffset = xOffset;
        m_yOffset = yOffset;
    }

    /**
     * Creates a camera angles class with no peramters, this is purely for testing and sim
     */
    public CameraAngles() {
        m_downwardAngleDegrees = 0;
        m_horzFOVDegrees = 0;
        m_vertFOVDegrees = 0;
        m_horzResolution = 0;
        m_vertResolution = 0;
        m_cameraHeightMeters = 0;
        m_xOffset = 0;
        m_yOffset = 0;
    }

    /**
     * @return A robot relative translational x value of an object in a camera in
     *         meters
     */
    public double getX(double vertPixels) {
        double x = -1.0 * m_cameraHeightMeters
                * Math.tan(vertPixels * (Math.toRadians(m_vertFOVDegrees / m_vertResolution))
                        + Math.toRadians(90 - m_vertFOVDegrees / 2 - m_downwardAngleDegrees));
        return x - m_xOffset;
    }

    /**
     * @return A robot relative translational y value of an object in a camera in
     *         meters
     */
    public double getY(double horzPixels, double vertPixels) {
        double x = getX(vertPixels);
        double y = -1.0 * x
                * Math.tan(Math.toRadians(m_horzFOVDegrees) * (horzPixels
                        - m_horzResolution / 2) / m_horzResolution);
        return y - m_yOffset;
    }
    /**
     * @return A robot relative angle in radians to the note
     */
    public double getAngle(double horzPixels, double vertPixels) {
            return Math.atan2(getY(horzPixels, vertPixels),getX(vertPixels));
    }
}