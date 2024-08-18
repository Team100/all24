# Factor Graph

A factor graph is an approach to approximate an arbitrary stochastic function.

The example here follows
https://github.com/tussedrotten/simple-factorgraph-example
which is licensed here
https://github.com/tussedrotten/simple-factorgraph-example/blob/main/LICENSE

this is itself based on
https://github.com/borglab/gtsam/blob/develop/python/gtsam/examples/PlanarSLAMExample.py
which is licensed here
https://github.com/borglab/gtsam/blob/develop/LICENSE.BSD

There's good stuff about matplotlib performance here

https://bastibe.de/2013-05-30-speeding-up-matplotlib.html

Note, you can run `isam.update()` as much as you want to get it to
make better estimates.

```
# for _ in range(5):
#     isam.update()
```

# Dependencies

```
scipy
pyqt5
gtsam
numpy
shapely
```

for vision simulation

```
pip install opencv-python
apt install python3-matplotlib
apt install libcairo2-dev
apt install python3-pyqt5
```