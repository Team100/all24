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

the apt package, python3-gtsam, appears to be the current 4.2 version, but it seems not to include gtsam_unstable.

the launchpad PPA appears to be stale, 4.0 from 2020, and has no version for ubuntu 24.

the pypi wheel says it only works with python 3.11, and ubuntu 24 uses python 3.12.

building from source fails

for vision simulation

```
pip install opencv-python
apt install python3-matplotlib
apt install libcairo2-dev
apt install python3-pyqt5
```


```
clone gtsam
apt install cmake
apt install libboost-all-dev
```

(skip TBB for now, come back to it if more cores need to be used)