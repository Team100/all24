# Use the official Raspberry Pi OS image as a base
FROM dtcooper/raspberrypi-os:bookworm

# Install system dependencies
RUN apt-get update
RUN apt-get install -y python3-pip
RUN apt-get install -y python3-setuptools
RUN apt-get install -y python3-wheel
RUN apt-get install -y python3-numpy
RUN apt-get install -y libcamera-dev
RUN apt-get install -y python3-aiohttp
RUN apt-get install -y python3-opencv
RUN apt-get install -y python3-venv

RUN apt-get clean  && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
# Create a virtual environment
RUN python3 -m venv /team100/venv

# Activate the virtual environment
ENV PATH="/team100/venv/bin:$PATH"

# Copy requirements file
COPY requirements.txt /tmp/requirements.txt

# Install dependencies in the virtual environment
# RUN pip3 install --no-cache-dir -r /tmp/requirements.txt
RUN pip3 install hidapi
RUN pip3 install adafruit-blinka
RUN pip3 install adafruit-circuitpython-lsm6ds
RUN pip3 install debugpy
RUN pip3 install numpy
RUN pip3 install aiohttp
RUN pip3 install opencv-python
# RUN pip3 install robotpy --prefer-binary
# RUN pip3 install robotpy-cscore --prefer-binary
# RUN pip3 install robotpy-apriltag --prefer-binary

# Create app directory
WORKDIR /team100

# Copy the app files
COPY app /team100/app
COPY app/config /team100/app/config

# Copy the runapp.py script
COPY runapp.py /team100/runapp.py

# Remove any __pycache__ directories
#RUN find /team100 -type d -name "__pycache__" -exec rm -rf {} +
EXPOSE 5678

# Set the entrypoint to run the app with debugger
#ENTRYPOINT ["python3", "-m", "debugpy", "--listen", "0.0.0.0:5678", "runapp.py"]
