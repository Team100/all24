# Visualization

Long ago I made some glass visualizations for mechanisms, but
we never used them, so they're now in this study.

Integration is in the relevant subsystem, e.g. ArmSubsystem.java:

```java
// a field
    private final ArmVisualization m_viz;

// in the constructor
        m_viz = new ArmVisualization(this);

// update it in periodic
    @Override
    public void periodic() {
        m_viz.viz();
    }

```

or SwerveModule100.java:

```java
// a field
    private final SwerveModuleVisualization m_viz;

// in the constructor
        m_viz = new SwerveModuleVisualization(this);

// in periodic
    /** Update visualization and logs. */
    void periodic() {
        m_viz.viz();
        m_driveServo.periodic();
        m_turningServo.periodic();
    }


```