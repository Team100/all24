# pylint: disable=missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement


import math
from turtle import Turtle
import random
import bisect
import time
import numpy as np
import cupy as cp  # type:ignore
from cupyx import jit  # type:ignore

print(f"CuPy version {cp.__version__}")
mempool = cp.get_default_memory_pool()
mempool.free_all_blocks()
print(f"mempool.used_bytes {mempool.used_bytes()}")


PARTICLE_COUNT = 1000
PARTICLES_TO_PLOT = 25
ROBOT_HAS_COMPASS = False
ROBOT_SPEED = 0.1
WIDTH = 5
HEIGHT = 5

RESAMPLE = True


# particle (x,y), Nx2
particles_xy = cp.random.uniform(
    low=(0, 0), high=(WIDTH, HEIGHT), size=(PARTICLE_COUNT, 2)
)

particles_h = cp.zeros(PARTICLE_COUNT)

# used temporarily by the resampler
new_particles_xy = cp.zeros_like(particles_xy)
indices = cp.zeros(PARTICLE_COUNT)

# particle weights (w), Nx1
particles_w = cp.zeros(PARTICLE_COUNT)

# beacon (x,y), Mx2
beacon_xy = cp.array([[0.5, 0.5], [0.5, 4.5]])

# particle-to-beacon distances (d), NxM
distances = cp.zeros((particles_xy.shape[0], beacon_xy.shape[0]))

# TODO: use this
# this is 2d so i can use it as a list-of-one beacon
robot_xy = cp.array([[WIDTH / 4, HEIGHT / 2]])
# robot-to-beacon distances (d), 1xM
robot_distances = cp.zeros((1, beacon_xy.shape[0]))


def init(turtle):
    turtle.screen.mode("standard")
    turtle.resizemode("user")
    turtle.screen.tracer(50000, delay=0)
    turtle.screen.register_shape("dot", ((-3, -3), (-3, 3), (3, 3), (3, -3)))
    turtle.screen.register_shape("tri", ((-3, -2), (0, 3), (3, -2), (0, 0)))
    turtle.speed(0)
    turtle.screen.title("particle filter demo")
    turtle.screen.setworldcoordinates(0, 0, WIDTH, HEIGHT)
    turtle.up()
    turtle.color("#00ffff")
    for x, y in beacon_xy:
        turtle.setposition(x, y)
        turtle.dot(10)
    turtle.screen.update()


def show_robot(turtle, x, y, h):
    turtle.color("green")
    turtle.shape("classic")
    turtle.shapesize(1)
    turtle.setposition(x, y)
    turtle.setheading(h)
    turtle.stamp()
    turtle.screen.update()


def weight_to_color(weight):
    if np.isnan(weight):
        weight = 0
    red = weight
    blue = 1 - weight
    return (red, 0, blue)


def show_particles(turtle):
    # turtle.shape("tri")
    turtle.shapesize(0.5)
    turtle.color("blue")
    turtle.shape("circle")
    for i, p in enumerate(particles_xy[:: PARTICLE_COUNT / PARTICLES_TO_PLOT]):
        turtle.setposition(p[0], p[1])
        turtle.setheading(particles_h[i])
        turtle.color(weight_to_color(particles_w[i].item()))
        turtle.stamp()


@jit.rawkernel()
def distance_kernel(
    particles: cp.ndarray,  # Nx2
    beacons: cp.ndarray,  # Mx2
    dist: cp.ndarray,  # NxM
    size: np.int32,  # this is N
) -> None:
    # pylint:disable=no-member
    tid = jit.blockIdx.x * jit.blockDim.x + jit.threadIdx.x
    ntid = jit.gridDim.x * jit.blockDim.x
    for i in range(tid, size, ntid):
        # i is the row in N
        for j in range(beacons.shape[0]):
            # j is the row in M
            p_x = particles[i, 0]
            p_y = particles[i, 1]
            b_x = beacons[j, 0]
            b_y = beacons[j, 1]
            dx = p_x - b_x
            dy = p_y - b_y
            d = cp.hypot(dx, dy)
            dist[i, j] = d


@jit.rawkernel()
def zero_out_of_bounds(
    particles: cp.ndarray, weights: cp.ndarray, size: np.int32
) -> None:
    # pylint:disable=no-member
    tid = jit.blockIdx.x * jit.blockDim.x + jit.threadIdx.x
    ntid = jit.gridDim.x * jit.blockDim.x
    for i in range(tid, size, ntid):
        p_x = particles[i, 0]
        p_y = particles[i, 1]
        if p_x < 0 or p_y < 0 or p_x > WIDTH or p_y > HEIGHT:
            weights[i] = 0


@jit.rawkernel()
def weigh_similar(
    particle_distances: cp.ndarray,
    robot_dist: cp.ndarray,
    weights: cp.ndarray,
    size: np.int32,
) -> None:
    # pylint:disable=no-member
    tid = jit.blockIdx.x * jit.blockDim.x + jit.threadIdx.x
    ntid = jit.gridDim.x * jit.blockDim.x
    # loop over particles
    for i in range(tid, size, ntid):
        sqsum = 0.0
        # loop over beacons
        for j in range(particle_distances.shape[1]):
            p_d = particle_distances[i, j]
            r_d = robot_dist[0, j]
            diff_d = p_d - r_d
            sqsum += diff_d * diff_d
        weights[i] = cp.exp(-1.0 * sqsum / 0.1)


# populate the particle-to-beacon distance array.
def all_beacon_distance() -> None:
    # pylint:disable=no-value-for-parameter
    distance_kernel(
        (128,), (1024,), (particles_xy, beacon_xy, distances, PARTICLE_COUNT)
    )


# populate the robot-to-beacon distance array
def robot_beacon_distance() -> None:
    # pylint:disable=no-value-for-parameter
    distance_kernel(
        (128,), (1024,), (robot_xy, beacon_xy, robot_distances, PARTICLE_COUNT)
    )


def normalize(weights) -> cp.ndarray:
    wsum = cp.sum(weights)
    return weights / wsum


def reweight() -> None:
    # pylint:disable=no-value-for-parameter
    # populate distances
    all_beacon_distance()
    # populate robot_distances
    robot_beacon_distance()

    global particles_w

    weigh_similar(
        (128,), (1024,), (distances, robot_distances, particles_w, PARTICLE_COUNT)
    )
    zero_out_of_bounds((128,), (1024,), (particles_xy, particles_w, PARTICLE_COUNT))
    particles_w = normalize(particles_w)


def compute_mean() -> tuple[float, float]:
    xy = cp.average(particles_xy, axis=0, weights=particles_w)
    return xy[0], xy[1]


def show_mean(turtle, x, y) -> None:
    turtle.color("#000000")
    turtle.setposition(x, y)
    turtle.shape("circle")
    turtle.stamp()


def resample() -> None:
    global particles_xy
    global new_particles_xy
    global indices

    indices = cp.random.choice(PARTICLE_COUNT, size=PARTICLE_COUNT, p=particles_w)
    new_particles_xy = cp.take(particles_xy, indices, axis=0)
    xynoise = cp.random.uniform(
        low=(-0.1, -0.1), high=(0.1, 0.1), size=(PARTICLE_COUNT, 2)
    )
    particles_xy = new_particles_xy + xynoise


def main():
    turtle = Turtle()
    init(turtle)
    print(f"mempool.used_bytes {mempool.used_bytes()}")

    robot_xy[0, 0] = WIDTH / 4
    robot_xy[0, 1] = HEIGHT / 2
    robot_h = 270

    loop_counter = 0

    while True:
        loop_counter += 1
        t0 = time.time_ns()

        reweight()
        # time.sleep(0.1)
        x, y = compute_mean()

        if loop_counter % 5 == 0:
            turtle.clearstamps()
            show_particles(turtle)
            show_mean(turtle, x, y)
            show_robot(turtle, robot_xy[0, 0], robot_xy[0, 1], robot_h)

        if RESAMPLE:
            resample()

        old_heading = robot_h

        robot_h += 5
        r = math.radians(robot_h)
        robot_xy[0, 0] += math.cos(r) * ROBOT_SPEED
        robot_xy[0, 1] += math.sin(r) * ROBOT_SPEED

        d_h = robot_h - old_heading

        if RESAMPLE:

            # just mirror the robot heading for now
            global particles_h
            global particles_xy

            particles_h = cp.ones((PARTICLE_COUNT, 1)) * robot_h
            r = cp.deg2rad(particles_h)
            dx = cp.cos(r) * ROBOT_SPEED
            dy = cp.sin(r) * ROBOT_SPEED
            d = cp.hstack((dx, dy))
            particles_xy += d

        t1 = time.time_ns()
        duration = t1 - t0
        if loop_counter % 2 == 0:
            print(f"duration (us): {duration//1000}")
            print(f"duration per particle (us): {duration//(1000*PARTICLE_COUNT)}")


if __name__ == "__main__":
    main()
