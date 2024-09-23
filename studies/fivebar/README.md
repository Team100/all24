# Five Bar

A two-dimensional parallel manipulator linkage, similar to a parallel SCARA machine.

The analysis uses these coordinates; a5 is the grounded link.  Note the images below are inverted compared to this diagram.

<img src="pantograph.png" width=600 />

These are examples of where the mechanism can reach in the work envelope.

<img src="python/examples.png" width=600 />

This shows that the entire envelope can be reached.

<img src="python/reach.png" width=600 />

This is the full envelope of the machine; notice the poor resolution near the edges.

<img src="python/envelope.png" width=600 />

This is the minimum force corresponding to stall torque.

<img src="python/force.png" width=600 />

This is the condition number of the Jacobian, indicating that the mechanism is well behaved.

<img src="python/condition.png" width=600 />

This repo is a copy of truher/fivebar, it's ok to let it diverge.