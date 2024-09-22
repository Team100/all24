# Slippery Tires

Over the summer I wrote some logic to decouple wheel rotation from odometry.

The basic idea is to limit actual acceleration when calculating the pose, so,
for example, a locked-wheel "hockey stop" results in maximum deceleration, but
not *infinite* deceleration.

More documentation here:

https://docs.google.com/document/d/1a-yI4T2AifRgIW3QJcWiV6-kvuVHfvHLoRkeGFjYLiI/edit

We never found time to test or tune it, so I removed it to this study.

Integration is a bit complicated.

Experiment.java:
```java
/**
 * Use slippery tire model
 */
SlipperyTires,
```

SwerveDrivePoseEstimator.java:
```java
// member:
private final SlipperyTireUtil m_tireUtil;
...

// constructor:
m_tireUtil = new SlipperyTireUtil(child, m_kinodynamics.getTire());
...

// logic requires the pose history to make available a temporally-consistent pair of
// poses, to calculate the actual velocity, and then this pair is used to adjust
// module position delta calculated in put():
if (Experiments.instance.enabled(Experiment.SlipperyTires) && consistentPair.size() > 1) {
    // get an earlier pose in order to adjust the corner velocities
    Map.Entry<Double, InterpolationRecord> earlierEntry = consistentPair.get(1);

    t0 = lowerEntry.getKey() - earlierEntry.getKey();
    final double t00 = t0;
    m_log_t0.log( () -> t00);
    earlierPose = earlierEntry.getValue().m_state;
    Vector2d[] corners = m_tireUtil.cornerDeltas(
            m_kinodynamics.getKinematics(),
            earlierPose.pose(),
            previousPose.pose());
    final SwerveModulePosition100[] delta0 = modulePositionDelta;
    m_log_delta0.log( () -> delta0[0]);
    modulePositionDelta = m_tireUtil.adjust(corners, t0, modulePositionDelta, t1);
    final SwerveModulePosition100[] delta1 = modulePositionDelta;
    m_log_delta1.log( () -> delta1[0]);
}

```

SwerveKinodynamics.java:
```java
// member
private Tire m_tire; // passed to ctor
...

// getter
public Tire getTire() {
    return m_tire;
}
```

