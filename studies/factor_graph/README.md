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

# Building GTSAM

This relies on pre-release GTSAM features, so you have to build it, and to do that, you need to make a few changes.

First check it out.

```
git clone git@github.com:borglab/gtsam.git
cd gtsam
```

Then edit cmake/HandleGeneralOptions.cmake.

* Since it's not necessary, turn GTSAM_WITH_TBB off.
* Since we use Python, turn GTSAM_BUILD_PYTHON on.

Then edit cmake/HandleGlobalBuildFlags.cmake

add this line to the bottom.  it works around an error that i think is unrelated to gtsam per se
it seems related to type_traits and the boost::serialization.

```
add_compile_options("-fpermissive")
```

Finally, edit python/CMakeLists.txt.  add "--break-system-packages" to the line that invokes ```pip install```.

These changes are also included in the branch called ```team100_install```

Then follow the normal instructions:

```
mkdir build
cd build
cmake ..
make -j6 check
make install
cd ../python
cmake ..
make -j6 python-install
```

instead of "6" above, use whatever your number of cores is.