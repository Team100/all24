package org.team100.lib.util;

import java.util.function.Supplier;

import org.team100.lib.localization.NotePosition24ArrayListener;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class CameraAngles {
    double m_downwardAngleDegrees;
    double m_horzFOVDegrees;
    double m_vertFOVDegrees;
    double m_horzResolution;
    double m_vertResolution;
    double m_cameraHeightMeters;
    Supplier<Pose2d> m_robotPose;
    NotePosition24ArrayListener m_notePosition24ArrayListener;
    public CameraAngles(
            double downwardAngleDegrees,
            double horzFOVDegrees,
            double vertFOVDegrees,
            double horzResolution,
            double vertResolution,
            double cameraHeightMeters,
            NotePosition24ArrayListener notePosition24ArrayListener,
            Supplier<Pose2d> robotPose) {
        m_downwardAngleDegrees = downwardAngleDegrees;
        m_horzFOVDegrees = horzFOVDegrees;
        m_vertFOVDegrees = vertFOVDegrees;
        m_horzResolution = horzResolution;
        m_vertResolution = vertResolution;
        m_cameraHeightMeters = cameraHeightMeters;
        m_notePosition24ArrayListener = notePosition24ArrayListener;
    }

    public CameraAngles(){
        m_downwardAngleDegrees = 0;
        m_horzFOVDegrees = 0;
        m_vertFOVDegrees = 0;
        m_horzResolution = 0;
        m_vertResolution = 0;
        m_cameraHeightMeters = 0;
        m_notePosition24ArrayListener = new NotePosition24ArrayListener();
    }

    /**
     * @return A robot relative translational x value of an object in a camera in meters
     */
    public Double getX() {
        if (m_notePosition24ArrayListener.getY() != null) {
        double x = m_cameraHeightMeters * Math.tan(m_notePosition24ArrayListener.getY() * (Math.toRadians(m_vertFOVDegrees / m_vertResolution))
                + Math.toRadians(90 - m_vertFOVDegrees - m_downwardAngleDegrees));
        return x;
        }
        return null;
    }

    /**
     * @return A robot relative translational y value of an object in a camera in meters
     */
    public Double getY() {
        if (m_notePosition24ArrayListener.getY() != null && m_notePosition24ArrayListener.getX() != null) {
        double x = getX();
        double y = x
                * Math.tan(Math.toRadians(m_horzFOVDegrees) * (m_notePosition24ArrayListener.getX() - m_horzResolution / 2) / m_horzResolution);
        return y;
    }
    return null;
}

    /**
     * @return A robot relative translational y value of an object in a camera in meters
     */
    public Double getY(double horzPixels, double vertPixels) {
    double x = getX(vertPixels);
    double y = x
            * Math.tan(Math.toRadians(m_horzFOVDegrees) * (horzPixels
             - m_horzResolution / 2) / m_horzResolution);
    return y;
}

    /**
     * @return A robot relative translational x value of an object in a camera in meters
     */
    public Double getX(double vertPixels) {
        double x = m_cameraHeightMeters * Math.tan(vertPixels * (Math.toRadians(m_vertFOVDegrees / m_vertResolution))
                + Math.toRadians(90 - m_vertFOVDegrees - m_downwardAngleDegrees));
        return x;
    }

    /**
     * @return A robot relative translational value of an object in a camera in meters
     */
    public Translation2d Translation2d() {
        if (m_notePosition24ArrayListener.getY() != null && m_notePosition24ArrayListener.getX() != null) {
        return new Translation2d(getX(), getY());
        }
        return null;
    }

    /**
     * @return A robot relative Transform2d value of an object in a camera in meters for translation and radians for rotation
     */
    public Transform2d robotRelativeTransform2d() {
        if (m_notePosition24ArrayListener.getY() != null && m_notePosition24ArrayListener.getX() != null) {
        return new Transform2d(getX(), getY(), new Rotation2d(getX(),getY()));
        }
        return null;
    }

    /**
     * @return A field relative Pose2d value of an object in a camera in meters for translation and radians for rotation
     */
    public Pose2d fieldRelativePose2d() {
        if (m_notePosition24ArrayListener.getY() != null && m_notePosition24ArrayListener.getX() != null) {
            return m_robotPose.get().transformBy(robotRelativeTransform2d());
        }
            return null;
    }
}