package org.team100.lib.util;

public class CameraAngles {
    private final double m_downwardAngleRads;
    private final double m_horzAngle = 0;
    private final double m_cameraHeightMeters;
    private final double m_xOffset;
    private final double m_yOffset;

    /**
     * @param downwardAngleRads The angle downward the camera is in
     *                             radians, this is including half of the
     *                             view of the camera
     * @param cameraHeightMeters   The height of the camera in meters
     * @param xOffset              The x offset of the camera in meters relative to
     *                             the center of the drivetrain
     * @param yOffset              The y offset of the camera in meters relative to
     *                             the center of the drivetrain
     */
    public CameraAngles(
            double downwardAngleRads,
            double cameraHeightMeters,
            double xOffset,
            double yOffset) {
        m_downwardAngleRads = downwardAngleRads;
        m_cameraHeightMeters = cameraHeightMeters;
        m_xOffset = xOffset;
        m_yOffset = yOffset;
    }

    /**
     * Creates a camera angles class with no peramters, this is purely for testing
     * and sim
     */
    public CameraAngles() {
        m_downwardAngleRads = 0;
        m_cameraHeightMeters = 0;
        m_xOffset = 0;
        m_yOffset = 0;
    }

    /**
     *         0 should be center of FOV
     * @return A robot relative translational x value of an object in a camera in
     *         meters
     */
    public double getX(double vertFOVRads) {
        double x = -1.0 * m_cameraHeightMeters
                * Math.tan(vertFOVRads + Math.PI/2 - m_downwardAngleRads);
        return x - m_xOffset;
    }

    /**
     *         0 should be center of FOV 
     * @return A robot relative translational y value of an object in a camera in
     *         meters
     */
    public double getY(double vertFOVRads, double horzFOVRads) {
        double x = -1.0 * getX(vertFOVRads);
        double y = x * Math.tan(horzFOVRads + m_horzAngle);
        return y - m_yOffset;
    }

    // /**
    //  * @return A camera vertical pixel measurement given a translational x value
    //  */
    // public double getInverseX(double forwardMeters) {
    //     double x = (m_vertResolution * (Math.atan((forwardMeters + m_xOffset) / m_cameraHeightMeters)
    //             - Math.toRadians(90 - m_vertFOVDegrees/2 - m_downwardAngleRads)))
    //             / Math.toRadians(m_vertFOVDegrees);
    //     return x;
    // }

    // /**
    //  * @return A camera horizontal pixel measurement given an x and y translational value, 0 is 416 pixels, or centerscreen
    //  */
    // public double getInverseY(double forwardMeters, double sideMeters) {
    //     double y = m_horzResolution * (Math.atan((sideMeters+m_yOffset)/(forwardMeters+m_xOffset))/Math.toRadians(m_horzFOVDegrees)) + m_horzResolution/2;
    //     return y;
    // }

    /**
     * @return A robot relative angle in radians to the note
     */
    public double getAngle(double horzFOVRads, double vertFOVRads) {
        return Math.atan2(getY(horzFOVRads, vertFOVRads), getX(vertFOVRads));
    }
}