# Install and Run on Ubuntu

To install on Ubuntu:

First install the pupil-apriltags python wrapper (which also installs apriltags itself):

```
python3 -m pip install pupil-apriltags
```

Then make a fork of github.com/Team100/main2023 into your own account, and then clone it somewhere on your workstation:

```
cd <somewhere handy>
git clone git@github.com:<your account>/main2023.git
cd main2023/apriltags_example
```

To see detections in a single image:

```
python3 pic.py
```

To see detections in your webcam's video stream:

```
python3 vid.py
```
