# Installation instructions

Execute these from the "terminal" window of vscode on your laptop, or on the Raspberry Pi.

## Apt packages:

```
sudo apt install python3-pip
sudo apt install python3-setuptools
sudo apt install python3-wheel
sudo apt install python3-numpy
sudo apt install libcamera-dev
sudo apt install picamera2
sudo apt install python3-aiohttp
```

## Python packages:

on the pi, these should be installed as sudo apt install python3-<thing>

```
python3 -m pip install numpy
python3 -m pip install robotpy
python3 -m pip install robotpy-cscore
python3 -m pip install robotpy-apriltag
python3 -m pip install opencv-python
```

## Gyro

```
python3 -m pip install hidapi
python3 -m pip install adafruit-blinka
python3 -m pip install adafruit-circuitpython-lsm6ds
```
