# Type Checking

I set up mypy on vscode, which highlights type issues in python.

To get it to understand all the things, I needed to install a few libraries on my desktop:

* python3 -m pip install --upgrade robotpy
* python3 -m pip install robotpy-cscore
* python3 -m pip install robotpy-apriltag
* sudo apt install python3-prctl
* python3 -m pip install picamera2

You can read about python type annotations [here.](https://docs.python.org/3/library/typing.html)