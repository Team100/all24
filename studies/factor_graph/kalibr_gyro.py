# pylint: disable=C0103,C0114,C0115,C0116,E0611,E1101,R0904,R0913,W0621
# mypy: disable-error-code="import-untyped"

# example code from https://github.com/ethz-asl/kalibr/issues/354#issuecomment-979934812

import numpy as np
import matplotlib.pyplot as plt

# n: number of measurements
# dt: sampling rate [s], e.g. dt = 1e-2 when sampling at 100 Hz
def generate_signal(n, dt, noise_density, random_walk, random_state=0):
    rng = np.random.RandomState(random_state)
    white = noise_density / np.sqrt(dt) * rng.randn(n)
    walk = random_walk * np.sqrt(dt) * np.cumsum(rng.randn(n))
    return white + walk



def main() -> None:
    x = generate_signal(100, 0.02, 0.01, 0.1)
    plt.plot(x)
    plt.show()


if __name__ == "__main__":
    main()
