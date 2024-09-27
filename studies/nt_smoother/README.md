# NT Smoother

This is an implementation of GTSAM fixed-lag smoothing, using WPI Network Tables as the input and output surfaces.

There's a simulator that provides Network Tables input, and a renderer that shows the Network Tables output pose.

Simulated inputs are camera sightings (realistically lagged), odometry, and yaw rate.

See studies/factor_graph for details about GTSAM, and some other demos.
