import math
import scipy

# number of teams
N = 50
# fudge factor
a = 1.07
print(f"Rank,Score")
for R in range(N):
    score = math.ceil(
        scipy.special.erfinv((N - 2 * R + 2) / (N * a))
        * (10 / scipy.special.erfinv(1 / a))
        + 12
    )
    print(f"{R},{score}")
