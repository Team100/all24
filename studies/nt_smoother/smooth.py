# fuses inputs fron network tables and publishes the result
# you want to run this concurrently with sim.py if you
# want a simulated "gym" to operate in

from ntcore import NetworkTableInstance


def main() -> None:
    # the fusor is an NT client; the RIO is the server.
    inst = NetworkTableInstance.getDefault()
    inst.setServer("localhost") # for now
    inst.startClient4("smooth")

if __name__ == "__main__":
    main()
