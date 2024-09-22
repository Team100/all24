# Camera Zoom

This is Vasili's project to implement forensic zooming in the camera subsystems.

The general idea is that the robot would know where the targets "should" appear, and the cameras would crop and zoom to those areas.

It was never really finished, so it's now in this study.

The CameraUpdater appeared in RobotContainer:

```java
//
    private final CameraUpdater m_cameraUpdater;

...

        m_cameraUpdater = new CameraUpdater(
                () -> m_drive.getState().pose(),
                m_layout);

...

    public void periodic() {
        if (Experiments.instance.enabled(Experiment.UseCameraUpdater))
            m_cameraUpdater.update();
    }
```

Experiment.java

```java
//
    /**
     * Periodically publish all tag poses to all cameras
     */
    UseCameraUpdater,
```
