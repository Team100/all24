############################################
#
# Sample code for the LSM6DSOX gyro.
#
# The deployment mode for this hardware would be
# using the I2C of one or more Raspberry Pi's,
# and doing the integration there.
#
# the uncorrected drift of this sensor is high,
# so there will need to be a "calibration" process
# of some kind, e.g. notice when the robot is not
# moving, and measure the offset, or do it once
# at startup, or something.
#



import time
import board
import busio
from adafruit_lsm6ds import Rate
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import matplotlib.pyplot as plt

# the offset seems to vary a bit depending on data rate
# offset = -0.01485 for 1.6khz
offset = -0.014935  # for 100hz
# a match is 2:30 long so 150 sec
sampletime = 150

i2c = board.I2C()
# this seems to make no difference
#i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
sox = LSM6DSOX(i2c)
# rate values are in adafruit_lsm6ds/__init__.py
sox.gyro_data_rate = Rate.RATE_104_HZ

print("Gyro rate set to: %d HZ" % Rate.string[sox.gyro_data_rate])

times = []
# the sample duration using the MCP2221 is 15ms, unacceptable
#durations = []
zs = []
yaws = []
yaw = 0;
maxyaw = 0
minyaw = 0
initialtime = time.time()
while True:
    starttime = time.time()
    z = sox.gyro[2] - offset
    endtime = time.time()
    t = endtime - initialtime
    duration = endtime - starttime
    dz = z * duration
    yaw += dz
    times.append(t)
    #durations.append(duration)
    zs.append(z)
    yaws.append(yaw)
    if (yaw > maxyaw):
        maxyaw = yaw
    if (yaw < minyaw):
        minyaw = yaw
    #print(f"{dt:.3f} {z:.4f}")
    #time.sleep(0.1)
    if (t > sampletime):
        break

print(f"max yaw {maxyaw:.5f} rad")
print(f"min yaw {minyaw:.5f} rad")
print(f"drift rate {yaw/sampletime:.5f} rad/s")
#print(durations)
plt.plot(times, zs)
plt.plot(times, yaws)
plt.xlabel("seconds")
plt.ylabel("radians")
plt.show()
