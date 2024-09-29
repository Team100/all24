# simulates the "gym" the robot is operating in.
# you want to run this concurrently with smooth.py

from ntcore import NetworkTableInstance


def main() -> None:
    # this impersonates the RIO as the NT server
    inst = NetworkTableInstance.getDefault()
    inst.startServer()

if __name__ == "__main__":
    main()
