package org.team100.lib.util;


import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class CameraAngles {
    private final double m_downwardAngleDegrees;
    private final double m_horzFOVDegrees;
    private final double m_vertFOVDegrees;
    private final double m_horzResolution;
    private final double m_vertResolution;
    private final double m_cameraHeightMeters;
    private final SwerveDriveSubsystem m_robot;
    private final NotePosition24ArrayListener m_notePosition24ArrayListener;
    private final Telemetry t = Telemetry.get();

    public CameraAngles(
            double downwardAngleDegrees,
            double horzFOVDegrees,
            double vertFOVDegrees,
            double horzResolution,
            double vertResolution,
            double cameraHeightMeters,
            NotePosition24ArrayListener notePosition24ArrayListener,
            SwerveDriveSubsystem robot) {
        m_downwardAngleDegrees = downwardAngleDegrees;
        m_horzFOVDegrees = horzFOVDegrees;
        m_vertFOVDegrees = vertFOVDegrees;
        m_horzResolution = horzResolution;
        m_vertResolution = vertResolution;
        m_cameraHeightMeters = cameraHeightMeters;
        m_notePosition24ArrayListener = notePosition24ArrayListener;
        m_robot = robot;
    }

    /**
     * @return A robot relative translational x value of an object in a camera in meters
     */
    public Double getX() {
        if (m_notePosition24ArrayListener.getY() != null) {
        double x = -1.0 * m_cameraHeightMeters * Math.tan(m_notePosition24ArrayListener.getY().get() * (Math.toRadians(m_vertFOVDegrees / m_vertResolution))
                + Math.toRadians(90 - m_vertFOVDegrees - m_downwardAngleDegrees));
                t.log(Level.DEBUG, "Camera Angles", "x: ", x);
                return x;
        }
        return null;
    }

    /**
     * @return A robot relative translational x value of an object in a camera in meters
     */
    public double getX(double vertPixels) {
        double x = -1.0 * m_cameraHeightMeters * Math.tan(vertPixels * (Math.toRadians(m_vertFOVDegrees / m_vertResolution))
                + Math.toRadians(90 - m_vertFOVDegrees - m_downwardAngleDegrees));
        return x;
    }

     /**
     * @return A robot relative angle in radians to the note
     */
    public double getAngleToNote() {
        return -1.0 * (Math.toRadians(m_horzFOVDegrees) * (m_notePosition24ArrayListener.getX().get() - m_horzResolution / 2) / m_horzResolution);
    }

    /**
     * @return A robot relative translational y value of an object in a camera in meters
     */
    public Double getY() {
        if (m_notePosition24ArrayListener.getY() != null && m_notePosition24ArrayListener.getX() != null) {
        double x = getX();
        double y = -1.0 * x
                * Math.tan(Math.toRadians(m_horzFOVDegrees) * (m_notePosition24ArrayListener.getX().get() - m_horzResolution / 2) / m_horzResolution);
                t.log(Level.DEBUG, "Camera Angles", "y: ", y);
                return y;
    }
    return null;
}

    /**
     * @return A robot relative translational y value of an object in a camera in meters
     */
    public double getY(double horzPixels, double vertPixels) {
    double x = getX(vertPixels);
    double y = x
            * Math.tan(Math.toRadians(m_horzFOVDegrees) * (horzPixels
             - m_horzResolution / 2) / m_horzResolution);
    return y;
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
        return new Transform2d(getX() + 1, getY(), new Rotation2d(-1.0 * (Math.toRadians(m_horzFOVDegrees) * (m_notePosition24ArrayListener.getX().get() - m_horzResolution / 2) / m_horzResolution)));
        }
        return null;
    }

    /**
     * @return A field relative Pose2d value of an object in a camera in meters for translation and radians for rotation
     */
    public Pose2d fieldRelativePose2d() {
        if (m_notePosition24ArrayListener.getY() != null && m_notePosition24ArrayListener.getX() != null) {
            return m_robot.getPose().transformBy(robotRelativeTransform2d());
        }
            return null;
    }
}